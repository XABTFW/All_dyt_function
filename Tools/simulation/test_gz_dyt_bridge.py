#!/usr/bin/env python3
"""Check image signs, upward rig singularity and joint limits without Gazebo."""
import math
import unittest
from types import SimpleNamespace

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V

from gimbal_pose_inspector import Camera, PoseStream, from_rpy, multiply, rotate
from gz_dyt_bridge import CameraPoseStream, DytBridge, camera_rpy_frd, check_target_texture, follow_step, tracking_measurement


def rig(yaw=0, roll=0, pitch=0):
    mount = from_rpy(0, 0, 0)
    yaw_q = from_rpy(0, 0, yaw)
    roll_q = multiply(yaw_q, from_rpy(roll, 0, 0))
    camera_q = multiply(roll_q, from_rpy(0, pitch-math.pi/2, 0))
    return [mount, yaw_q, roll_q, camera_q]


class BridgeTest(unittest.TestCase):
    def test_upward_euler_branch_preserves_all_camera_axes(self):
        for roll in (-2.9, -.2, 0, 1.7):
            for pitch in (-math.pi/2, -1.53, -1.61, .4):
                for yaw in (-3.1, -2.2, 0, 2.2, 3.1):
                    flu = from_rpy(roll, pitch, yaw)
                    frd = from_rpy(*camera_rpy_frd(flu))
                    self.assertLessEqual(abs(camera_rpy_frd(flu)[2]), math.pi/2+1e-8)
                    for axis in ((1,0,0), (0,1,0), (0,0,1)):
                        original = rotate(from_rpy(roll, -pitch, -yaw), axis)
                        np.testing.assert_allclose(rotate(frd, axis), original, atol=1e-7)

    def test_transport_packet_timestamp_frame_interval_and_invalidation(self):
        bridge = DytBridge.__new__(DytBridge)
        bridge.camera = Camera(1280, 720, 2, .05, 15000, ((0, 0, 0), (1, 0, 0, 0)))
        bridge.follow, bridge.last_stamp = False, None
        bridge.last_yaw_command = None
        bridge.keys = ["model", "mount", "yaw", "roll", "camera"]
        bridge.names = dict(zip(bridge.keys, range(1, 6)))
        bridge.parents = {1: None, 2: 1, 3: 1, 4: 1, 5: 1}
        bridge.stream = PoseStream()
        packets = []
        bridge.publisher = SimpleNamespace(publish=lambda packet: packets.append(packet) or True)
        frame = Pose_V()
        frame.header.stamp.sec = 12
        for identifier, quaternion in enumerate([from_rpy(0, 0, 0)]+rig(), 1):
            pose = frame.pose.add(id=identifier)
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = quaternion
        bridge.stream.receive(frame)
        image = Image(width=1280, height=720)
        image.header.stamp.sec = 12
        target = {"valid": True, "center_px": [639.5, 359.5], "bbox_xywh": [630, 350, 20, 20]}
        bridge.update(image, target)
        self.assertEqual(len(packets[0].data), 12)
        self.assertEqual(packets[0].header.stamp.sec, 12)
        self.assertAlmostEqual(packets[0].data[5], math.pi/2, places=6)
        self.assertEqual(packets[0].data[11], 0)
        bridge.update(image, target)
        self.assertEqual(len(packets), 1)
        image.header.stamp.nsec = frame.header.stamp.nsec = 100_000_000
        bridge.stream.receive(frame)
        bridge.update(image, target)
        self.assertAlmostEqual(packets[-1].data[11], .1)
        bridge.invalidate()
        self.assertEqual(list(packets[-1].data), [1]+[0]*11)
        image.header.stamp.nsec = 300_000_000
        with self.assertRaisesRegex(ValueError, "frame_gap"):
            bridge.update(image, target)

    def test_uniform_background_cannot_guide_gimbal(self):
        image = np.full((20, 20, 3), (0, 0, 200), dtype=np.uint8)
        target = {"valid": True, "bbox_xywh": [0, 0, 20, 20]}
        with self.assertRaisesRegex(ValueError, "no_spatial_texture"):
            check_target_texture(image, target)
        image[::2, ::2] = 255
        check_target_texture(image, target)

    def test_pixel_center_and_right_down(self):
        target = {"valid": True, "center_px": [639.5, 359.5], "bbox_xywh": [630, 350, 20, 20]}
        self.assertEqual(tracking_measurement(target, 1280, 720, 2)[:2], [0, 0])
        target["center_px"] = [739.5, 409.5]
        lx, ly, *_ = tracking_measurement(target, 1280, 720, 2)
        self.assertGreater(lx, 0)
        self.assertGreater(ly, 0)
        self.assertAlmostEqual(math.tan(lx)/math.tan(ly), 2)
        self.assertEqual(tracking_measurement({"valid": False}, 1280, 720, 2), [0]*6)

    def test_follow_reduces_error_and_respects_rate(self):
        for yaw, roll, pitch in [(0, 0, 0), (0.7, 0.2, -0.8), (0, 0, -2.1)]:
            for lx, ly in [(0.15, 0), (0, 0.15), (-0.1, -0.1)]:
                with self.subTest(yaw=yaw, pitch=pitch, lx=lx, ly=ly):
                    qs = rig(yaw, roll, pitch)
                    desired = np.array(rotate(qs[-1], (1, -math.tan(lx), -math.tan(ly))))
                    desired /= np.linalg.norm(desired)
                    angles = follow_step(qs, lx, ly, 0.1)
                    old = np.array(rotate(qs[-1], (1, 0, 0)))
                    new = np.array(rotate(rig(*angles)[-1], (1, 0, 0)))
                    self.assertGreater(float(new @ desired), float(old @ desired))
                    self.assertLessEqual(max(abs(a-b) for a, b in zip(angles, [yaw, roll, pitch])), 0.050001)

    def test_center_holds_measured_angles_including_large_negative_pitch(self):
        for pitch in [0, -0.8, -1.8, -2.35619, 0.785398]:
            np.testing.assert_allclose(follow_step(rig(0.3, 0.2, pitch), 0, 0, 0.1),
                                       [0.3, 0.2, pitch], atol=1e-7)

    def test_center_pixel_noise_does_not_move_joints(self):
        angles = follow_step(rig(.4, .2, -.8), .002, -.003, 1/30)
        np.testing.assert_allclose(angles, [.4, .2, -.8], atol=1e-7)

    def test_pose_history_matches_exposure_and_rejects_old_image(self):
        stream = CameraPoseStream()
        for ns in (0, 30_000_000, 60_000_000, 100_000_000):
            frame = Pose_V()
            frame.header.stamp.sec, frame.header.stamp.nsec = 12, ns
            stream.receive(frame)
        self.assertEqual(stream.at_image(12_030_000_000).header.stamp.nsec, 30_000_000)
        with self.assertRaisesRegex(ValueError, "time_mismatch"):
            stream.at_image(12_150_000_000)
        frame.header.stamp.sec = 13
        stream.receive(frame)
        with self.assertRaisesRegex(ValueError, "max_age"):
            stream.at_image(12_030_000_000)

    def test_mechanical_limits(self):
        angles = follow_step(rig(0, 0.785398, -2.35619), -0.5, 0.5, 0.1)
        self.assertLessEqual(abs(angles[1]), 0.785398)
        self.assertGreaterEqual(angles[2], -2.35619)
        self.assertLessEqual(angles[2], 0.785398)

    def test_yaw_wrap_keeps_continuous_joint_command(self):
        angles = follow_step(rig(-math.pi+.01, 0, -.8), 0, 0, .1, math.pi+.01)
        self.assertAlmostEqual(angles[0], math.pi+.01)


if __name__ == "__main__":
    unittest.main()
