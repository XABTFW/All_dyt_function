#!/usr/bin/env python3

import csv
import math
from collections import deque
from datetime import datetime, timedelta, timezone

import numpy as np
from pyulog import ULog


LOG_PATH = "/home/btfw/下载/log_27_2026-9-16-12-02-52.ulg"
OUTPUT_PATH = "log_27_net_capture_fusion_100ms.csv"

IMAGE_DISTANCE_MIN_M = 1.0
IMAGE_DISTANCE_MAX_M = 50.0
IMAGE_LONG_DISTANCE_SCALE = 1.2075
IMAGE_LONG_DISTANCE_OFFSET = 0.6356
IMAGE_DISTANCE_JUMP_MIN_M = 0.8
IMAGE_DISTANCE_JUMP_REL = 0.2
IMAGE_DISTANCE_JUMP_SPEED_SCALE = 2.0
IMAGE_DISTANCE_REINIT_COUNT = 3
IMAGE_SPEED_WINDOW_US = 250_000
IMAGE_SPEED_MIN_SPAN_US = 150_000
IMAGE_SPEED_MAX_GAP_US = 250_000
IMAGE_SPEED_MAX_AGE_US = 500_000
IMAGE_CLOSING_SPEED_MAX_M_S = 75.0
FUSION_LASER_MAX_AGE_US = 50_000
FUSION_CORRECTION_ALPHA = 0.15
FUSION_DISTANCE_SCALE_MIN = 0.5
FUSION_DISTANCE_SCALE_MAX = 1.5
FUSION_TRACK_DISTANCE_GATE_M = 1.5
FUSION_SPEED_SCALE_MIN = 0.5
FUSION_SPEED_SCALE_MAX = 1.5
FUSION_SPEED_GATE_M_S = 5.0


def finite(value):
	return math.isfinite(float(value))


class Replay:
	def __init__(self):
		self.raw_history = deque(maxlen=3)
		self.last_raw_time = 0
		self.last_accepted_time = 0
		self.last_accepted_distance = math.nan
		self.reject_candidate = math.nan
		self.reject_count = 0
		self.speed_history = deque(maxlen=64)
		self.speed_last_sample_time = 0
		self.image_speed = math.nan
		self.image_speed_valid = False
		self.image_speed_timestamp = 0

		self.distance_scale = math.nan
		self.distance_scale_valid = False
		self.initial_distance_scale = math.nan
		self.initial_laser_count = 0
		self.speed_scale = math.nan
		self.speed_scale_valid = False
		self.initial_speed_scale = math.nan
		self.initial_speed_count = 0
		self.last_fusion_laser_time = 0

		self.laser_time = 0
		self.laser_distance = math.nan
		self.laser_speed = math.nan
		self.laser_velocity_time = 0
		self.laser_velocity_distance = math.nan

	def clear_image_filter(self):
		self.raw_history.clear()
		self.last_raw_time = 0
		self.last_accepted_time = 0
		self.last_accepted_distance = math.nan
		self.reject_candidate = math.nan
		self.reject_count = 0

	def clear_speed_history(self):
		self.speed_history.clear()
		self.speed_last_sample_time = 0

	def reset_fusion(self):
		self.distance_scale = math.nan
		self.distance_scale_valid = False
		self.initial_distance_scale = math.nan
		self.initial_laser_count = 0
		self.speed_scale = math.nan
		self.speed_scale_valid = False
		self.initial_speed_scale = math.nan
		self.initial_speed_count = 0
		self.last_fusion_laser_time = 0

	def invalidate_image(self):
		self.clear_image_filter()
		self.clear_speed_history()
		self.reset_fusion()
		self.image_speed = math.nan
		self.image_speed_valid = False
		self.image_speed_timestamp = 0

	def update_laser(self, timestamp, distance):
		if not finite(distance) or distance < 0.05 or distance >= 50.0:
			return

		update_baseline = self.laser_velocity_time == 0 or not finite(self.laser_velocity_distance)

		if not update_baseline:
			dt = (timestamp - self.laser_velocity_time) * 1e-6

			if 0.02 <= dt <= 2.5:
				raw_speed = (self.laser_velocity_distance - distance) / dt

				if finite(raw_speed) and abs(raw_speed) <= 50.0:
					alpha = dt / (0.3 + dt)
					self.laser_speed = (self.laser_speed + alpha * (raw_speed - self.laser_speed)
								if finite(self.laser_speed) else raw_speed)

				update_baseline = True

			elif dt > 2.5:
				self.laser_speed = math.nan
				update_baseline = True

		if update_baseline:
			self.laser_velocity_time = timestamp
			self.laser_velocity_distance = distance

		self.laser_time = timestamp
		self.laser_distance = distance

	def filter_image_distance(self, timestamp, raw_distance):
		if timestamp == 0 or not finite(raw_distance):
			return None

		if self.last_raw_time and (timestamp <= self.last_raw_time
								or timestamp - self.last_raw_time > IMAGE_SPEED_MAX_GAP_US):
			self.clear_image_filter()

		self.last_raw_time = timestamp
		self.raw_history.append(raw_distance)

		if len(self.raw_history) < 3:
			return None

		median_distance = float(np.median(np.asarray(self.raw_history)))
		raw_limit = max(IMAGE_DISTANCE_JUMP_MIN_M, IMAGE_DISTANCE_JUMP_REL * abs(median_distance))

		if self.last_accepted_time and timestamp >= self.last_accepted_time and self.image_speed_valid:
			dt = (timestamp - self.last_accepted_time) * 1e-6
			raw_limit = max(raw_limit, IMAGE_DISTANCE_JUMP_SPEED_SCALE * min(max(self.image_speed, 0.0), 75.0) * dt)

		if abs(raw_distance - median_distance) > raw_limit:
			return None

		if not finite(self.last_accepted_distance) or self.last_accepted_time == 0:
			self.last_accepted_distance = raw_distance
			self.last_accepted_time = timestamp
			return raw_distance

		dt = (timestamp - self.last_accepted_time) * 1e-6
		previous_speed = min(max(self.image_speed, 0.0), 75.0) if self.image_speed_valid else 0.0
		predicted_distance = self.last_accepted_distance - previous_speed * dt
		jump_limit = max(IMAGE_DISTANCE_JUMP_MIN_M,
						 IMAGE_DISTANCE_JUMP_REL * abs(self.last_accepted_distance),
						 IMAGE_DISTANCE_JUMP_SPEED_SCALE * previous_speed * dt)
		jump_valid = finite(predicted_distance) and finite(jump_limit) and abs(median_distance - predicted_distance) <= jump_limit

		if not jump_valid:
			consistency_limit = max(0.5, IMAGE_DISTANCE_JUMP_REL * abs(median_distance))
			consistent = (self.reject_count > 0 and finite(self.reject_candidate)
						  and abs(median_distance - self.reject_candidate) <= consistency_limit)

			if consistent:
				self.reject_count += 1
				self.reject_candidate = 0.5 * (self.reject_candidate + median_distance)
			else:
				self.reject_count = 1
				self.reject_candidate = median_distance

			if self.reject_count < IMAGE_DISTANCE_REINIT_COUNT:
				return None

			self.clear_speed_history()
			self.reset_fusion()

		self.last_accepted_distance = raw_distance
		self.last_accepted_time = timestamp
		self.reject_candidate = math.nan
		self.reject_count = 0
		return raw_distance

	def estimate_image_speed(self):
		if len(self.speed_history) < 3:
			return None

		latest_time = self.speed_history[-1][0]
		samples = [(timestamp, distance) for timestamp, distance in self.speed_history
				   if timestamp <= latest_time and latest_time - timestamp <= IMAGE_SPEED_WINDOW_US and finite(distance)]

		if len(samples) < 3 or latest_time - samples[0][0] < IMAGE_SPEED_MIN_SPAN_US:
			return None

		times = np.asarray([-(latest_time - timestamp) * 1e-6 for timestamp, _ in samples], dtype=float)
		distances = np.asarray([distance for _, distance in samples], dtype=float)
		denominator = len(samples) * float(np.dot(times, times)) - float(times.sum()) ** 2

		if not finite(denominator) or denominator < 1e-6:
			return None

		distance_rate = (len(samples) * float(np.dot(times, distances))
						 - float(times.sum()) * float(distances.sum())) / denominator
		return -distance_rate

	def fuse(self, now, image_distance):
		result = {
			"fused_range_valid": False,
			"fused_speed_valid": False,
			"laser_fusion_used": False,
			"fused_distance": math.nan,
			"fused_speed": math.nan,
			"laser_distance_compensated": math.nan,
			"laser_speed": math.nan,
			"distance_scale": self.distance_scale,
			"speed_scale": self.speed_scale,
		}

		if not self.image_speed_valid:
			return result

		laser_fresh = self.laser_time and self.laser_time <= now and now - self.laser_time <= FUSION_LASER_MAX_AGE_US
		laser_measurement_valid = laser_fresh and finite(self.laser_distance) and 0.05 <= self.laser_distance <= 50.0
		new_laser = laser_measurement_valid and self.laser_time != self.last_fusion_laser_time
		laser_prediction_speed_valid = finite(self.laser_speed) and 0.0 <= self.laser_speed <= 75.0
		laser_for_fusion = self.laser_distance

		if laser_measurement_valid and laser_prediction_speed_valid:
			laser_for_fusion -= self.laser_speed * ((now - self.laser_time) * 1e-6)

		laser_distance_valid = laser_measurement_valid and finite(laser_for_fusion) and 0.05 <= laser_for_fusion <= 50.0

		if laser_distance_valid:
			result["laser_distance_compensated"] = laser_for_fusion
			raw_scale = self.laser_distance / image_distance
			raw_scale_valid = finite(raw_scale) and FUSION_DISTANCE_SCALE_MIN <= raw_scale <= FUSION_DISTANCE_SCALE_MAX

			if not self.distance_scale_valid and new_laser and raw_scale_valid:
				if self.initial_laser_count == 0:
					self.initial_distance_scale = raw_scale
					self.initial_laser_count = 1
				else:
					self.initial_distance_scale = 0.5 * (self.initial_distance_scale + raw_scale)
					self.initial_laser_count += 1

					if self.initial_laser_count >= 2:
						self.distance_scale = self.initial_distance_scale
						self.distance_scale_valid = True

			elif not self.distance_scale_valid and new_laser:
				self.initial_laser_count = 0
				self.initial_distance_scale = math.nan

			corrected_image = image_distance * self.distance_scale if self.distance_scale_valid else image_distance

			if (self.distance_scale_valid and raw_scale_valid
					and abs(corrected_image - laser_for_fusion) <= FUSION_TRACK_DISTANCE_GATE_M):
				if new_laser:
					self.distance_scale += FUSION_CORRECTION_ALPHA * (raw_scale - self.distance_scale)

				result["fused_distance"] = laser_for_fusion
				result["laser_fusion_used"] = True

		if not result["laser_fusion_used"]:
			result["fused_distance"] = image_distance * self.distance_scale if self.distance_scale_valid else image_distance

		laser_speed_valid = result["laser_fusion_used"] and finite(self.laser_speed) and 0.0 <= self.laser_speed <= 75.0

		if laser_speed_valid:
			result["laser_speed"] = self.laser_speed
			raw_speed_scale = self.laser_speed / self.image_speed if self.image_speed >= 0.1 else math.nan
			raw_speed_scale_valid = finite(raw_speed_scale) and FUSION_SPEED_SCALE_MIN <= raw_speed_scale <= FUSION_SPEED_SCALE_MAX

			if not self.speed_scale_valid and new_laser and raw_speed_scale_valid:
				if self.initial_speed_count == 0:
					self.initial_speed_scale = raw_speed_scale
					self.initial_speed_count = 1
				else:
					self.initial_speed_scale = 0.5 * (self.initial_speed_scale + raw_speed_scale)
					self.initial_speed_count += 1

					if self.initial_speed_count >= 2:
						self.speed_scale = self.initial_speed_scale
						self.speed_scale_valid = True

			elif not self.speed_scale_valid and new_laser:
				self.initial_speed_count = 0
				self.initial_speed_scale = math.nan

			corrected_image_speed = self.image_speed * self.speed_scale if self.speed_scale_valid else self.image_speed

			if self.speed_scale_valid and abs(corrected_image_speed - self.laser_speed) <= FUSION_SPEED_GATE_M_S:
				if new_laser and raw_speed_scale_valid:
					self.speed_scale += FUSION_CORRECTION_ALPHA * (raw_speed_scale - self.speed_scale)

				result["fused_speed"] = self.laser_speed

		if new_laser:
			self.last_fusion_laser_time = self.laser_time

		if not finite(result["fused_speed"]):
			result["fused_speed"] = self.image_speed * self.speed_scale if self.speed_scale_valid else self.image_speed

		result["fused_range_valid"] = finite(result["fused_distance"]) and 0.05 <= result["fused_distance"] <= 50.0
		result["fused_speed_valid"] = finite(result["fused_speed"]) and 0.0 <= result["fused_speed"] <= 75.0
		result["distance_scale"] = self.distance_scale
		result["speed_scale"] = self.speed_scale
		return result

	def update(self, timestamp, width, height, laser_range):
		if finite(laser_range):
			self.update_laser(timestamp, float(laser_range))

		if not finite(width) or not finite(height) or width < 4.0 or height < 4.0 or width > 1920 or height > 1080:
			self.invalidate_image()
			return None

		long_ratio = max(float(width), float(height)) / 1920.0
		raw_distance = IMAGE_LONG_DISTANCE_SCALE / long_ratio - IMAGE_LONG_DISTANCE_OFFSET

		if not finite(raw_distance) or raw_distance < IMAGE_DISTANCE_MIN_M or raw_distance > IMAGE_DISTANCE_MAX_M:
			self.invalidate_image()
			return None

		filtered_distance = self.filter_image_distance(timestamp, raw_distance)

		if filtered_distance is None:
			return None

		if self.speed_last_sample_time and (timestamp <= self.speed_last_sample_time
								 or timestamp - self.speed_last_sample_time > IMAGE_SPEED_MAX_GAP_US):
			self.clear_speed_history()

		self.speed_history.append((timestamp, filtered_distance))
		self.speed_last_sample_time = timestamp
		speed = self.estimate_image_speed()
		self.image_speed_valid = speed is not None and 0.0 <= speed <= IMAGE_CLOSING_SPEED_MAX_M_S
		self.image_speed = speed if self.image_speed_valid else math.nan
		self.image_speed_timestamp = timestamp if self.image_speed_valid else 0
		fusion = self.fuse(timestamp, filtered_distance)
		fusion.update({
			"timestamp": timestamp,
			"image_range_valid": True,
			"image_distance": filtered_distance,
			"image_speed_valid": self.image_speed_valid,
			"image_speed": self.image_speed,
		})
		return fusion


def format_value(value, digits=3):
	return "" if value is None or not finite(value) else f"{float(value):.{digits}f}"


def main():
	ulog = ULog(LOG_PATH)
	target = next(dataset for dataset in ulog.data_list if dataset.name == "dyt_target").data
	timestamps = target["timestamp_sample"].astype(np.int64)
	locked = (target["tracking_state"] == 1) & (target["target_valid"] == 1)

	# Select the final continuous lock interval containing the real gripper release.
	loss_indices = np.flatnonzero(locked[:-1] & ~locked[1:]) + 1
	loss_index = int(loss_indices[-1])
	lock_start_candidates = np.flatnonzero(~locked[:loss_index] & locked[1:loss_index + 1]) + 1
	lock_start_index = int(lock_start_candidates[-1])
	loss_time = int(timestamps[loss_index])

	replay = Replay()
	outputs = []

	for index in range(lock_start_index, loss_index):
		result = replay.update(int(timestamps[index]), float(target["bbox_width_px"][index]),
						   float(target["bbox_height_px"][index]), float(target["range_m"][index]))

		if result is not None:
			outputs.append(result)

	if not outputs:
		raise RuntimeError("No valid image-ranging samples found in the final lock interval")

	start_time = int(outputs[0]["timestamp"])
	grid_times = list(range(start_time, loss_time, 100_000))

	if not grid_times or grid_times[-1] != loss_time:
		grid_times.append(loss_time)

	boot_utc_us = int(ulog.msg_info_dict["boot_time_utc_us"])
	china_tz = timezone(timedelta(hours=8))
	rows = []
	output_index = 0
	last_output = None

	for grid_time in grid_times:
		while output_index < len(outputs) and int(outputs[output_index]["timestamp"]) <= grid_time:
			last_output = outputs[output_index]
			output_index += 1

		is_loss = grid_time == loss_time
		utc_time = datetime.fromtimestamp((boot_utc_us + grid_time) / 1e6, tz=timezone.utc)
		local_time = utc_time.astimezone(china_tz)
		row = {
			"relative_time_s": f"{(grid_time - start_time) * 1e-6:.3f}",
			"boot_time_s": f"{grid_time * 1e-6:.6f}",
			"local_time": local_time.isoformat(timespec="milliseconds"),
			"lock_valid": 0 if is_loss else 1,
			"image_range_valid": 0 if is_loss or last_output is None else 1,
			"image_distance_m": "" if is_loss or last_output is None else format_value(last_output["image_distance"]),
			"image_speed_valid": 0 if is_loss or last_output is None else int(last_output["image_speed_valid"]),
			"image_closing_speed_m_s": "" if is_loss or last_output is None else format_value(last_output["image_speed"]),
			"fused_range_valid": 0 if is_loss or last_output is None else int(last_output["fused_range_valid"]),
			"fused_distance_m": "" if is_loss or last_output is None else format_value(last_output["fused_distance"]),
			"fused_speed_valid": 0 if is_loss or last_output is None else int(last_output["fused_speed_valid"]),
			"fused_closing_speed_m_s": "" if is_loss or last_output is None else format_value(last_output["fused_speed"]),
			"laser_fusion_used": 0 if is_loss or last_output is None else int(last_output["laser_fusion_used"]),
			"laser_distance_compensated_m": "" if is_loss or last_output is None else format_value(last_output["laser_distance_compensated"]),
			"laser_closing_speed_m_s": "" if is_loss or last_output is None else format_value(last_output["laser_speed"]),
			"image_distance_scale": "" if is_loss or last_output is None else format_value(last_output["distance_scale"], 6),
			"image_speed_scale": "" if is_loss or last_output is None else format_value(last_output["speed_scale"], 6),
			"event": "seeker_lock_lost" if is_loss else ("image_range_first_valid" if grid_time == start_time else ""),
		}
		rows.append(row)

	with open(OUTPUT_PATH, "w", newline="", encoding="utf-8-sig") as output_file:
		writer = csv.DictWriter(output_file, fieldnames=list(rows[0].keys()))
		writer.writeheader()
		writer.writerows(rows)

	valid_fusion = [result for result in outputs if result["fused_range_valid"] and result["fused_speed_valid"]]
	laser_used = [result for result in outputs if result["laser_fusion_used"]]
	print(f"lock_start={timestamps[lock_start_index] * 1e-6:.6f}s")
	print(f"image_valid_start={start_time * 1e-6:.6f}s")
	print(f"fusion_valid_start={valid_fusion[0]['timestamp'] * 1e-6:.6f}s" if valid_fusion else "fusion_valid_start=none")
	print(f"laser_fusion_start={laser_used[0]['timestamp'] * 1e-6:.6f}s" if laser_used else "laser_fusion_start=none")
	print(f"laser_fusion_end={laser_used[-1]['timestamp'] * 1e-6:.6f}s" if laser_used else "laser_fusion_end=none")
	print(f"lock_loss={loss_time * 1e-6:.6f}s")
	print(f"rows={len(rows)} output={OUTPUT_PATH}")


if __name__ == "__main__":
	main()
