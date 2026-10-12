#!/usr/bin/env python3
"""Gazebo camera tracking; optional DYT SITL telemetry and gimbal following."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10 import image_pb2


def timestamp_ns(message):
    if not message.HasField("header") or not message.header.HasField("stamp"):
        raise ValueError("missing_image_timestamp")
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nsec < 1_000_000_000:
        raise ValueError("invalid_image_timestamp")
    return stamp.sec*1_000_000_000+stamp.nsec


def decode_image(message):
    """Respect per-row padding; return a new contiguous OpenCV BGR image."""
    formats = {
        image_pb2.RGB_INT8: (3, cv2.COLOR_RGB2BGR),
        image_pb2.BGR_INT8: (3, None),
        image_pb2.RGBA_INT8: (4, cv2.COLOR_RGBA2BGR),
        image_pb2.BGRA_INT8: (4, cv2.COLOR_BGRA2BGR),
        image_pb2.L_INT8: (1, cv2.COLOR_GRAY2BGR),
    }
    if message.pixel_format_type not in formats:
        raise ValueError("unsupported_pixel_format:"+str(message.pixel_format_type))
    channels, conversion = formats[message.pixel_format_type]
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width*channels:
        raise ValueError("invalid_image_dimensions_or_step")
    if len(message.data) != step*height:
        raise ValueError("image_payload_length_mismatch")
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    pixels = np.ascontiguousarray(pixels[:, :width*channels]).reshape(height, width, channels)
    if conversion is not None:
        return cv2.cvtColor(pixels, conversion)
    return pixels.copy()


class ImageStream:
    """Keep only the newest received image; slow consumers do not build a queue."""

    def __init__(self):
        self.lock = threading.Lock()
        self.message = None
        self.received_at = None
        self.advanced_at = None
        self.stamp = None
        self.sequence = 0
        self.epoch = 0
        self.error = None

    def receive(self, message):
        now = time.monotonic()
        try:
            stamp = timestamp_ns(message)
            error = None
        except ValueError as exception:
            stamp, error = None, str(exception)
        with self.lock:
            self.sequence += 1
            self.message, self.received_at, self.error = message, now, error
            if error is None:
                if self.stamp is None or stamp != self.stamp:
                    if self.stamp is not None and stamp < self.stamp:
                        self.epoch += 1
                    self.advanced_at = now
                self.stamp = stamp

    def snapshot(self, max_age):
        with self.lock:
            if self.message is None:
                raise ValueError("waiting_for_image")
            if self.error is not None:
                raise ValueError(self.error)
            age = time.monotonic()-self.received_at
            if age > max_age:
                raise ValueError("stale_image")
            if time.monotonic()-self.advanced_at > max_age:
                raise ValueError("image_timestamp_not_advancing")
            return self.message, self.sequence, self.epoch, age


def positive_float(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def positive_int(value):
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return result


def match_threshold(value):
    result = float(value)
    if not math.isfinite(result) or not 0 < result <= 1:
        raise argparse.ArgumentTypeError("must be in (0, 1]")
    return result


def parse_roi(value):
    try:
        x, y, width, height = map(int, value.split(","))
    except ValueError as exception:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height") from exception
    if x < 0 or y < 0 or width < 4 or height < 4:
        raise argparse.ArgumentTypeError("ROI needs nonnegative x/y and width/height >= 4")
    return x, y, width, height


class TargetTracker:
    """Local scale/rotation matching of the selected, immutable appearance.

    Foreground boundaries and texture validation reject uniform sky matches.
    A missed match never becomes a valid target measurement.
    """

    # Bound all scale/rotation searches, including near-field targets filling the image.
    MATCH_MAX_SIDE = 256

    def __init__(self, auto_lock=True, min_area=100, threshold=0.8, lost_timeout=2.0):
        self.auto_lock = auto_lock
        self.min_area = min_area
        self.threshold = threshold
        self.lost_timeout = lost_timeout
        self.next_id = 1
        self.reset()

    def reset(self):
        self.background = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=25,
                                                          detectShadows=False)
        self.frames = 0
        self.shape = None
        self.template = None
        self.template_mask = None
        self.scale = 1.0
        self.angle = 0.0
        self.box = None
        self.target_id = None
        self.last_seen = None
        self.state = "detecting"

    def lock_target(self, image, box, stamp_s):
        x, y, width, height = map(int, box)
        if x < 0 or y < 0 or width < 4 or height < 4 or \
                x+width > image.shape[1] or y+height > image.shape[0]:
            raise ValueError("target_roi_out_of_image")
        template = image[y:y+height, x:x+width].copy()
        # Normalized squared difference is undefined for an all-black template.
        if not np.any(template):
            raise ValueError("target_roi_has_no_appearance")
        self.template, self.box = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY), (x, y, width, height)
        # Ignore uniform background when the ROI includes a small aircraft against sky.
        border = np.concatenate((template[0], template[-1], template[:, 0], template[:, -1]))
        background = np.median(border, axis=0)
        foreground = (np.max(np.abs(template.astype(float)-background), axis=2) > 20).astype(np.uint8)*255
        self.template_mask = (cv2.dilate(foreground, np.ones((7, 7), np.uint8))
                              if 0.02 <= np.mean(foreground > 0) <= 0.8 else None)
        self.scale = 1.0
        self.angle = 0.0
        self.target_id = self.next_id
        self.next_id += 1
        self.last_seen = stamp_s
        self.state = "locked"

    def appearance(self, scale, angle):
        height, width = self.template.shape
        if abs(angle) < 1e-6:
            size = (max(4, round(width*scale)), max(4, round(height*scale)))
            mask = None if self.template_mask is None else cv2.resize(
                self.template_mask, size, interpolation=cv2.INTER_NEAREST)
            return cv2.resize(self.template, size, interpolation=(cv2.INTER_AREA if scale < 1.0
                                                                  else cv2.INTER_LINEAR)), mask
        radians = math.radians(angle)
        size = (max(4, round(scale*(width*abs(math.cos(radians))+height*abs(math.sin(radians))))),
                max(4, round(scale*(height*abs(math.cos(radians))+width*abs(math.sin(radians))))))
        transform = cv2.getRotationMatrix2D(((width-1)/2, (height-1)/2), angle, scale)
        transform[:, 2] += [(size[0]-width)/2, (size[1]-height)/2]
        template = cv2.warpAffine(self.template, transform, size)
        original_mask = self.template_mask if self.template_mask is not None else np.full_like(self.template, 255)
        mask = cv2.warpAffine(original_mask, transform, size, flags=cv2.INTER_NEAREST)
        return template, mask

    def update(self, image, stamp_s, roi=None):
        if self.shape is not None and self.shape != image.shape:
            self.reset()
        self.shape = image.shape
        if roi is not None:
            self.lock_target(image, roi, stamp_s)
        self.frames += 1
        candidates = []
        mask = self.background.apply(image) if self.template is None else None
        # MOG2 marks the entire initial frame as foreground. Warm up first.
        if self.frames > 10 and mask is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                x, y, width, height = cv2.boundingRect(contour)
                if self.min_area <= cv2.contourArea(contour) <= image.shape[0]*image.shape[1]*0.5 \
                        and width >= 4 and height >= 4 and np.any(image[y:y+height, x:x+width]):
                    candidates.append((x, y, width, height))
            candidates.sort(key=lambda box: box[2]*box[3], reverse=True)

        score = None
        valid = False
        if roi is not None:
            score, valid = 1.0, True
        elif self.template is None and self.state == "detecting" and self.auto_lock and candidates:
            self.lock_target(image, candidates[0], stamp_s)
            score, valid = 1.0, True
        elif self.template is not None and self.state != "lost":
            elapsed = stamp_s-self.last_seen
            if elapsed > self.lost_timeout:
                self.state = "lost"
            else:
                x, y, width, height = self.box
                margin = int(max(24, min(64, max(width, height)*0.35))*(1+elapsed/self.lost_timeout))
                left, top = max(0, x-margin), max(0, y-margin)
                right = min(image.shape[1], x+width+margin)
                bottom = min(image.shape[0], y+height+margin)
                search = cv2.cvtColor(image[top:bottom, left:right], cv2.COLOR_BGR2GRAY)
                match_scale = min(1.0, self.MATCH_MAX_SIDE/max(search.shape))
                if match_scale < 1.0:
                    search = cv2.resize(search, None, fx=match_scale, fy=match_scale,
                                        interpolation=cv2.INTER_AREA)
                # Actual resize ratios include integer rounding of the search dimensions.
                ratio_x = search.shape[1]/(right-left)
                ratio_y = search.shape[0]/(bottom-top)
                best = None
                pixels = search.astype(np.float32)
                squared_pixels = pixels*pixels
                # Preserve the selected appearance, but follow its scale as range changes.
                scales = (self.scale, self.scale*0.92, self.scale*1.08)
                proposals = [(scale, self.angle) for scale in scales]
                fine_scales = (self.scale*0.96, self.scale*1.04)
                proposals.extend((scale, self.angle) for scale in fine_scales)
                proposals.extend((scale, self.angle+rotation) for rotation in (-2, 2, -4, 4, -8, 8)
                                 for scale in (*scales, *fine_scales))
                # Rotation fallback is used only when the current orientation no longer fits.
                for index, (scale, angle) in enumerate(proposals):
                    if best is not None and ((index == 3 and best[0] <= max(0.05, 1.0-self.threshold-0.03))
                                             or (index > 3 and best[0] <= 0.10)):
                        break
                    template, mask = self.appearance(scale*match_scale, angle)
                    th, tw = template.shape
                    if tw > search.shape[1] or th > search.shape[0]:
                        continue
                    matches = cv2.matchTemplate(search, template, cv2.TM_SQDIFF_NORMED, mask=mask)
                    matches[~np.isfinite(matches)] = 1.0
                    # A mostly-sky ROI must never turn the empty sky into a valid match.
                    mean = cv2.boxFilter(pixels, -1, (tw, th), anchor=(0, 0))
                    mean_square = cv2.boxFilter(squared_pixels, -1, (tw, th), anchor=(0, 0))
                    variance = (mean_square-mean*mean)[:matches.shape[0], :matches.shape[1]]
                    minimum_texture = max(3.0, float(np.std(template))*0.25)
                    matches[variance < minimum_texture**2] = 1.0
                    error, _, location, _ = cv2.minMaxLoc(matches)
                    if best is None or error < best[0]:
                        # Quadratic peak refinement avoids multi-pixel staircase motion
                        # when the search is reduced. Never refine across rejected pixels.
                        refined = list(map(float, location))
                        for axis in (0, 1):
                            ix, iy = location
                            if 0 < ix < matches.shape[1]-1 and 0 < iy < matches.shape[0]-1:
                                before = float(matches[iy-(axis == 1), ix-(axis == 0)])
                                after = float(matches[iy+(axis == 1), ix+(axis == 0)])
                                curvature = before+after-2*error
                                if before < 1.0 and after < 1.0 and curvature > 1e-6:
                                    refined[axis] += float(np.clip((before-after)/(2*curvature), -0.5, 0.5))
                        best = (error, refined, tw, th, scale, angle)
                score = 0.0 if best is None else max(0.0, min(1.0, 1.0-best[0]))
                if score >= self.threshold:
                    _, location, width, height, self.scale, self.angle = best
                    # Report full-image geometry, never reduced-image coordinates.
                    cx = left+(location[0]+width/2)/ratio_x
                    cy = top+(location[1]+height/2)/ratio_y
                    radians = math.radians(self.angle)
                    original_h, original_w = self.template.shape
                    width = max(4, round(self.scale*(original_w*abs(math.cos(radians))+
                                                   original_h*abs(math.sin(radians)))))
                    height = max(4, round(self.scale*(original_h*abs(math.cos(radians))+
                                                    original_w*abs(math.sin(radians)))))
                    self.box = (max(0, min(image.shape[1]-width, round(cx-width/2))),
                                max(0, min(image.shape[0]-height, round(cy-height/2))), width, height)
                    self.last_seen = stamp_s
                    self.state, valid = "tracking", True
                else:
                    self.state = "searching"

        result = {"state": self.state, "valid": valid, "target_id": self.target_id,
                  "bbox_xywh": list(self.box) if valid else None,
                  "match_score": score, "candidates_xywh": [list(box) for box in candidates]}
        if valid:
            x, y, width, height = self.box
            cx, cy = x+width/2, y+height/2
            result["center_px"] = [cx, cy]
            result["center_error_normalized"] = [2*cx/image.shape[1]-1, 2*cy/image.shape[0]-1]
        return result


def draw_tracking(preview, target):
    for x, y, width, height in target["candidates_xywh"]:
        cv2.rectangle(preview, (x, y), (x+width, y+height), (0, 200, 255), 1)
    if target["valid"]:
        x, y, width, height = target["bbox_xywh"]
        cv2.rectangle(preview, (x, y), (x+width, y+height), (0, 255, 0), 2)
        cx, cy = map(int, target["center_px"])
        cv2.drawMarker(preview, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
    label = f"target={target['target_id']} {target['state']}"
    cv2.putText(preview, label, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 255, 0) if target["valid"] else (0, 200, 255), 2)


def window_closed(window):
    try:
        visible = cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible == 0
        # GTK3 returns -1 for unsupported VISIBLE even while the window is open.
        return cv2.getWindowProperty(window, cv2.WND_PROP_ASPECT_RATIO) < 0
    except cv2.error:
        # Some backends raise when querying a window destroyed by its close button.
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list-topics", action="store_true", help="list actual Gazebo topics, then exit")
    parser.add_argument("--topic", help="exact image topic from gz topic -l; type must be gz.msgs.Image")
    parser.add_argument("--headless", action="store_true", help="log/save without opening a window")
    parser.add_argument("--record-dir", type=Path, help="create a NEW directory for PNGs and frames.jsonl")
    parser.add_argument("--rate", type=positive_float, default=60.0, help="maximum processing/saving wall Hz (default 60; camera is 30 Hz)")
    parser.add_argument("--max-age", type=positive_float, default=0.5, help="receive freshness limit, wall seconds")
    parser.add_argument("--duration", type=positive_float, help="stop after this many wall seconds")
    parser.add_argument("--manual-lock", action="store_true", help="detect but only lock a mouse/CLI ROI")
    parser.add_argument("--roi", type=parse_roi, help="lock first frame ROI: x,y,width,height (also headless)")
    parser.add_argument("--min-target-area", type=positive_int, default=100, help="minimum motion contour pixels")
    parser.add_argument("--match-threshold", type=match_threshold, default=0.8, help="minimum appearance match score")
    parser.add_argument("--lost-timeout", type=positive_float, default=2.0, help="abandon missing target after simulation seconds")
    parser.add_argument("--dyt-model", help="publish tracking to this fpv_quadtailsitter_gimbal SITL instance")
    parser.add_argument("--world", default="default", help="Gazebo world for DYT pose feedback")
    parser.add_argument("--follow-gimbal", action="store_true", help="with --dyt-model, center target using simulated gimbal joints")
    args = parser.parse_args()
    if not args.list_topics and not args.topic:
        parser.error("--topic is required unless --list-topics is used")
    if args.dyt_model and args.rate < 30:
        parser.error("DYT input requires --rate >= 30 to consume the 30 Hz camera")
    if args.follow_gimbal and not args.dyt_model:
        parser.error("--follow-gimbal requires --dyt-model")
    if not args.list_topics and not args.headless and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("no graphical session; use --headless")
    # Avoid large OpenCV thread pools causing latency spikes alongside Gazebo rendering.
    cv2.setNumThreads(1)
    from gz.transport13 import Node
    node = Node()
    manifest = None
    dyt_bridge = None
    window = "Gazebo camera - drag to lock, r to reset, q/Esc to exit"
    window_open = False
    try:
        if args.list_topics:
            # Allow discovery briefly; a topic name alone does not prove its type.
            time.sleep(0.5)
            print("\n".join(sorted(node.topic_list())))
            return 0
        if args.dyt_model:
            from gz_dyt_bridge import DytBridge, check_target_texture
            dyt_bridge = DytBridge(node, args.dyt_model, args.world, args.topic, args.follow_gimbal)
        if args.record_dir:
            args.record_dir.mkdir(parents=True, exist_ok=False)
            manifest = (args.record_dir/"frames.jsonl").open("x", encoding="utf-8")

        def emit(record):
            line = json.dumps(record, ensure_ascii=False, allow_nan=False)+"\n"
            sys.stdout.write(line)
            sys.stdout.flush()
            if manifest:
                manifest.write(line)
                manifest.flush()

        stream = ImageStream()
        if not node.subscribe(image_pb2.Image, args.topic, stream.receive):
            raise ValueError("image_subscription_failed")
        emit({"type": "metadata", "schema_version": 2, "image_topic": args.topic,
              "read_only": dyt_bridge is None, "dyt_model": args.dyt_model,
              "follow_gimbal": args.follow_gimbal, "saved_pixel_format": "BGR_INT8_as_PNG",
              "alpha_handling": "discarded_for_RGBA_BGRA", "geometry_pose_input": dyt_bridge is not None,
              "max_processing_rate_wall_hz": args.rate, "queue_policy": "latest_received_only",
              "timestamp_source": "gz.msgs.Image.header.stamp", "freshness": "local_receive_wall_time",
              "detector": "MOG2_motion", "tracker": "foreground_masked_scale_rotation_template",
              "matching_max_side_px": TargetTracker.MATCH_MAX_SIDE,
              "auto_lock": not args.manual_lock, "initial_roi": args.roi,
              "min_target_area": args.min_target_area, "match_threshold": args.match_threshold,
              "lost_timeout_sim_s": args.lost_timeout, "motion_warmup_frames": 10,
              "dyt_max_frame_gap_sim_s": 0.12, "dyt_max_image_age_sim_s": 0.25})
        tracker = TargetTracker(not args.manual_lock, args.min_target_area,
                                args.match_threshold, args.lost_timeout)
        pending_roi = args.roi
        drag_start = None

        def select_target(event, x, y, flags, userdata):
            nonlocal pending_roi, drag_start
            if event == cv2.EVENT_LBUTTONDOWN:
                drag_start = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and drag_start is not None:
                sx, sy = drag_start
                drag_start = None
                if abs(x-sx) >= 4 and abs(y-sy) >= 4:
                    pending_roi = (min(sx, x), min(sy, y), abs(x-sx), abs(y-sy))

        if not args.headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(window, select_target)
            window_open = True
        start = time.monotonic()
        last_sequence = 0
        last_state = None
        last_bridge_error = None
        dyt_timing = None
        processed = 0
        last_shape = (360, 640, 3)
        last_epoch = None
        last_stamp = None
        tracking_elapsed = 0.0
        while args.duration is None or time.monotonic()-start < args.duration:
            tick = time.monotonic()
            preview = None
            try:
                message, sequence, epoch, age = stream.snapshot(args.max_age)
                if sequence != last_sequence:
                    tracking_start = time.monotonic()
                    image = decode_image(message)
                    stamp = timestamp_ns(message)
                    if last_epoch is not None and epoch != last_epoch:
                        tracker.reset()
                        last_stamp = None
                    if tracker.shape is not None and tracker.shape != image.shape:
                        tracker.reset()
                        last_stamp = None
                    # Repeated transport messages with a frozen stamp are not new tracker frames.
                    if stamp != last_stamp:
                        try:
                            target = tracker.update(image, stamp/1e9, pending_roi)
                        except ValueError as exception:
                            if pending_roi is None:
                                raise
                            emit({"type": "selection_error", "error": str(exception)})
                            target = tracker.update(image, stamp/1e9)
                        tracking_elapsed = time.monotonic()-tracking_start
                        pending_roi = None
                        last_stamp = stamp
                        if dyt_bridge:
                            # Telemetry/pose outages must not erase the selected image identity.
                            try:
                                check_target_texture(image, target)
                                dyt_timing = dyt_bridge.update(message, target)
                                bridge_error = None
                            except ValueError as exception:
                                bridge_error = str(exception)
                                dyt_timing = None
                                dyt_bridge.invalidate()
                            if bridge_error != last_bridge_error:
                                emit({"type": "dyt_state", "valid": bridge_error is None,
                                      "error": bridge_error})
                                last_bridge_error = bridge_error
                    last_epoch = epoch
                    last_shape = image.shape
                    processed += 1
                    record = {"type": "frame", "valid": True, "sequence": sequence,
                              "processed_frames": processed, "sim_epoch": epoch,
                              "stamp_ns": stamp, "width": message.width,
                              "height": message.height, "source_step_bytes": message.step,
                              "source_pixel_format": image_pb2.PixelFormatType.Name(message.pixel_format_type),
                              "receive_age_wall_s": age, "wall_elapsed_s": tick-start,
                              "skipped_received_frames": sequence-last_sequence-1, "target": target,
                              "tracking_time_wall_s": tracking_elapsed,
                              "processing_time_wall_s": time.monotonic()-tick,
                              "dyt_timing": dyt_timing,
                              "dyt_valid": dyt_bridge is not None and target["valid"] and last_bridge_error is None}
                    if args.record_dir:
                        filename = f"frame_{sequence:09d}.png"
                        if not cv2.imwrite(str(args.record_dir/filename), image):
                            raise OSError("PNG write failed")
                        record["png"] = filename
                    emit(record)
                    last_sequence = sequence
                    # Saved PNGs contain pixels only. Overlay a separate display copy.
                    preview = image.copy()
                    cv2.putText(preview, f"seq={sequence} sim={record['stamp_ns']/1e9:.3f}s", (12, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                    draw_tracking(preview, target)
                state = "receiving_image"
            except ValueError as exception:
                state = str(exception)
                # Keep the template for recovery; publish no valid measurement while stale.
                if dyt_bridge:
                    dyt_bridge.invalidate()
                # Clear a stale display instead of presenting the last image as live.
                preview = np.zeros(last_shape, dtype=np.uint8)
                cv2.putText(preview, state, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if state != last_state:
                emit({"type": "stream_state", "valid": state == "receiving_image",
                      "state": state, "wall_elapsed_s": tick-start})
                print(state, file=sys.stderr)
                last_state = state
            if window_open:
                if preview is not None:
                    cv2.imshow(window, preview)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key == ord("r"):
                    tracker.reset()
                    pending_roi = None
                    last_stamp = None
                if window_closed(window):
                    break
            time.sleep(max(0.0, 1/args.rate-(time.monotonic()-tick)))
        return 0
    except (ValueError, OSError, cv2.error) as exception:
        print(f"error: {exception}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        if args.topic:
            node.unsubscribe(args.topic)
        if manifest:
            manifest.close()
        if dyt_bridge:
            dyt_bridge.close()
        if window_open:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
