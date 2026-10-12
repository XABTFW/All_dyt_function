"""Opt-in SITL camera tracking telemetry and fpv_quadtailsitter_gimbal following."""

import math
from collections import deque

import numpy as np
from gz.msgs10.double_pb2 import Double
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.float_v_pb2 import Float_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene

from gimbal_pose_inspector import (DEFAULT_SDF, PoseStream, compose, inverse,
                                   load_camera, multiply, rotate, scene_index,
                                   to_rpy_deg, world_pose)


class CameraPoseStream(PoseStream):
    """Match camera orientation to image exposure instead of using the latest pose."""
    def __init__(self):
        super().__init__()
        self.history = deque(maxlen=120)

    def receive(self, frame):
        stamp = frame.header.stamp.sec*1_000_000_000+frame.header.stamp.nsec
        with self.lock:
            if self.stamp is not None and stamp < self.stamp:
                self.history.clear()
            self.history.append((stamp, frame))
        super().receive(frame)

    def at_image(self, stamp):
        self.snapshot(0.25)
        with self.lock:
            latest = self.stamp
            nearest_stamp, frame = min(self.history, key=lambda entry: abs(entry[0]-stamp))
        if latest-stamp > 250_000_000:
            raise ValueError("dyt_image_exceeds_max_age_0.25s")
        if abs(nearest_stamp-stamp) > 20_000_000:
            raise ValueError("dyt_image_pose_time_mismatch")
        return frame


def camera_rpy_frd(relative):
    """Equivalent camera Euler angles with yaw in [-pi/2, pi/2].

    The upward camera crosses the Euler pitch singularity. Standard extraction
    can report yaw near pi even with a centred target, beyond DYT's yaw check.
    Use the equivalent Euler branch, preserving the full measured rotation.
    """
    w, x, qy, qz = relative
    frd = (w, x, -qy, -qz)  # FLU -> FRD on both sides
    axis_x, axis_y, axis_z = [rotate(frd, axis) for axis in ((1,0,0), (0,1,0), (0,0,1))]
    horizontal = math.hypot(axis_x[0], axis_x[1])
    p = math.atan2(-axis_x[2], horizontal)
    if horizontal < 1e-8:
        y = 0.0
        r = math.atan2(-axis_z[1], axis_y[1])
    else:
        r = math.atan2(axis_y[2], axis_z[2])
        y = math.atan2(axis_x[1], axis_x[0])
    if abs(y) > math.pi/2:
        wrap = lambda angle: (angle+math.pi) % (2*math.pi)-math.pi
        r, p, y = wrap(r+math.pi), wrap(math.pi-p), wrap(y+math.pi)
    return r, p, y


def tracking_measurement(target, width, height, hfov):
    """Pinhole angles, positive image right/down, matching DYT FRD LOS."""
    if not target["valid"]:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy = target["center_px"]
    dx, dy = cx-(width-1)/2, cy-(height-1)/2
    focal = width/(2*math.tan(hfov/2))
    _, _, bw, bh = target["bbox_xywh"]
    return [math.atan2(dx, focal), math.atan2(dy, focal), bw, bh, dx, dy]


def check_target_texture(image, target):
    if target["valid"]:
        x, y, width, height = target["bbox_xywh"]
        patch = image[y:y+height, x:x+width]
        # A constant sky/background template matches anywhere and cannot guide a gimbal.
        if max(np.std(patch, axis=(0, 1))) < 3:
            raise ValueError("dyt_target_has_no_spatial_texture")


def follow_step(quaternions, los_x, los_y, dt, previous_yaw=None):
    """Damped inverse Jacobian: yaw Z, roll X, pitch Y in their parent frames.

    Use measured link rotations, including the camera's fixed -pi/2 pitch.
    At the upward-looking yaw singularity, roll and pitch still center the image.
    Limit each joint to 0.5 rad/s and to the SDF roll/pitch mechanical limits.
    """
    mount, yaw, roll, camera = quaternions
    forward = np.array(rotate(camera, (1, 0, 0)))
    desired = np.array(rotate(camera, (1, -math.tan(los_x), -math.tan(los_y))))
    desired /= np.linalg.norm(desired)
    axes = [rotate(mount, (0, 0, 1)), rotate(yaw, (1, 0, 0)), rotate(roll, (0, 1, 0))]
    jacobian = np.column_stack([np.cross(axis, forward) for axis in axes])
    # Do not rotate yaw around an almost vertical optical axis to chase pixel noise.
    weights = np.diag([max(0.02, 1-forward[2]**2), 1.0, 1.0])
    delta = weights @ jacobian.T @ np.linalg.solve(jacobian @ weights @ jacobian.T+0.01*np.eye(3), desired-forward)
    delta *= min(1.0, 5.0*dt)  # proportional velocity, independent of processing frequency
    if math.hypot(los_x, los_y) < 0.005:  # about two camera pixels
        delta[:] = 0
    delta = np.clip(delta, -0.5*dt, 0.5*dt)
    # Fixed camera pitch is -pi/2. Remove it before extracting the pitch joint.
    yaw_angle = math.radians(to_rpy_deg(multiply(inverse(mount), yaw))[2])
    if previous_yaw is not None:
        yaw_angle += 2*math.pi*round((previous_yaw-yaw_angle)/(2*math.pi))
    roll_angle = math.radians(to_rpy_deg(multiply(inverse(yaw), roll))[0])
    relative_camera = multiply(inverse(roll), camera)
    axis_x = rotate(relative_camera, (1, 0, 0))
    fixed_pitch = math.atan2(-axis_x[2], axis_x[0])
    if fixed_pitch > 0:
        fixed_pitch -= 2*math.pi
    pitch_angle = fixed_pitch+math.pi/2
    return [yaw_angle+delta[0], float(np.clip(roll_angle+delta[1], -0.785398, 0.785398)),
            float(np.clip(pitch_angle+delta[2], -2.35619, 0.785398))]


class DytBridge:
    def __init__(self, node, model, world, image_topic, follow=False):
        # This trial supports the existing upward-facing tailsitter rig only.
        expected = f"/world/{world}/model/{model}/link/camera_link/sensor/camera/image"
        if image_topic != expected:
            raise ValueError("dyt_image_topic_does_not_match_model_camera:"+expected)
        self.camera = load_camera(DEFAULT_SDF, "camera_link", "camera")
        self.node, self.model, self.follow = node, model, follow
        self.publisher = node.advertise(f"/model/{model}/dyt/target", Float_V)
        ok, scene = node.request(f"/world/{world}/scene/info", Empty(), Empty, Scene, 500)
        if not ok:
            raise ValueError("dyt_scene_service_unavailable")
        self.parents, self.names = scene_index(scene)
        self.keys = [model, model+"::gimbal_mount_link", model+"::gimbal_yaw_link",
                     model+"::gimbal_roll_link", model+"::camera_link"]
        if any(key not in self.names for key in self.keys):
            raise ValueError("dyt_requires_fpv_quadtailsitter_gimbal_links")
        self.stream = CameraPoseStream()
        self.pose_topic = f"/world/{world}/pose/info"
        if not node.subscribe(Pose_V, self.pose_topic, self.stream.receive):
            raise ValueError("dyt_pose_subscription_failed")
        self.commands = [node.advertise(f"/model/{model}/command/gimbal_{axis}", Double)
                         for axis in ("yaw", "roll", "pitch")] if follow else []
        self.owner = node.advertise(f"/model/{model}/dyt/gimbal_owner", Boolean) if follow else None
        self.last_stamp = None
        self.last_yaw_command = None

    def invalidate(self):
        if self.last_stamp is not None:
            packet = Float_V(data=[1]+[0]*11)
            packet.header.stamp.sec, packet.header.stamp.nsec = divmod(self.last_stamp, 1_000_000_000)
            self.publisher.publish(packet)

    def update(self, message, target):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nsec
        if stamp == self.last_stamp:
            return
        sample_dt = max(0.0, (stamp-self.last_stamp)/1e9) if self.last_stamp is not None else 0.0
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.last_yaw_command = None
        dt = min(0.1, sample_dt)
        self.last_stamp = stamp
        if (message.width, message.height) != (self.camera.width, self.camera.height):
            raise ValueError("dyt_image_size_does_not_match_camera_sdf")
        if getattr(self, "owner", None):
            self.owner.publish(Boolean(data=True))
        if sample_dt > 0.12:
            raise ValueError("dyt_frame_gap_exceeds_0.12s")
        if hasattr(self.stream, "at_image"):
            frame = self.stream.at_image(stamp)
        else:
            frame, _, _, _ = self.stream.snapshot(0.25)
        pose_stamp = frame.header.stamp.sec*1_000_000_000+frame.header.stamp.nsec
        # Reject old/future pose feedback instead of mixing unrelated camera orientations.
        if abs(stamp-pose_stamp) > 20_000_000:
            raise ValueError("dyt_image_pose_time_mismatch")
        poses = {pose.id: pose for pose in frame.pose}
        transforms = [world_pose(self.names[key], self.parents, poses) for key in self.keys]
        sensor = compose(transforms[-1], self.camera.sensor_pose)
        relative = multiply(inverse(transforms[0][1]), sensor[1])
        r, p, y = camera_rpy_frd(relative)
        lx, ly, bw, bh, dx, dy = tracking_measurement(target, message.width, message.height, self.camera.hfov)
        packet = Float_V()
        # Gazebo's Python modules can expose distinct Time classes per message.
        packet.header.stamp.sec = message.header.stamp.sec
        packet.header.stamp.nsec = message.header.stamp.nsec
        packet.data.extend([1, int(target["valid"]), lx, ly, r, p, y, bw, bh, dx, dy, sample_dt])
        if not self.publisher.publish(packet):
            raise ValueError("dyt_publish_failed")
        if self.follow and target["valid"] and dt > 0 and math.hypot(lx, ly) >= 0.005:
            angles = follow_step([transform[1] for transform in transforms[1:]], lx, ly, dt, self.last_yaw_command)
            self.last_yaw_command = angles[0]
            for publisher, angle in zip(self.commands, angles):
                if not publisher.publish(Double(data=angle)):
                    raise ValueError("dyt_gimbal_command_failed")

        return {"frame_dt_sim_s": sample_dt,
                "pose_skew_sim_s": abs(stamp-pose_stamp)/1e9,
                "image_age_sim_s": max(0.0, (self.stream.stamp-stamp)/1e9)}

    def close(self):
        self.invalidate()
        if self.owner:
            self.owner.publish(Boolean(data=False))
        self.node.unsubscribe(self.pose_topic)
