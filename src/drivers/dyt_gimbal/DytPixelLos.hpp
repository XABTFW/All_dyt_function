/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <math.h>
#include <stdint.h>

namespace dyt
{

inline bool pixelMissToLos(int16_t miss_x_px, int16_t miss_y_px, float horizontal_fov_deg,
			  int32_t image_width_px, int32_t image_height_px, float manual_scale_deg_px,
			  float &los_x_rad, float &los_y_rad)
{
	los_x_rad = NAN;
	los_y_rad = NAN;

	if (image_width_px <= 0 || image_height_px <= 0) {
		return false;
	}

	const int32_t miss_x = miss_x_px;
	const int32_t miss_y = miss_y_px;

	if (miss_x < -(image_width_px / 2) || miss_x > image_width_px / 2 ||
	    miss_y < -(image_height_px / 2) || miss_y > image_height_px / 2) {
		return false;
	}

	if (isfinite(manual_scale_deg_px) && manual_scale_deg_px > 0.f) {
		constexpr float DEG_TO_RAD = 0.01745329251994329577f;
		los_x_rad = static_cast<float>(miss_x) * manual_scale_deg_px * DEG_TO_RAD;
		los_y_rad = static_cast<float>(miss_y) * manual_scale_deg_px * DEG_TO_RAD;
		return isfinite(los_x_rad) && isfinite(los_y_rad);
	}

	if (!isfinite(horizontal_fov_deg) || horizontal_fov_deg <= 0.f || horizontal_fov_deg >= 179.f) {
		return false;
	}

	constexpr float DEG_TO_RAD = 0.01745329251994329577f;
	const float horizontal_fov_rad = horizontal_fov_deg * DEG_TO_RAD;
	const float focal_length_px = 0.5f * static_cast<float>(image_width_px) / tanf(0.5f * horizontal_fov_rad);

	if (!isfinite(focal_length_px) || focal_length_px <= 0.f) {
		return false;
	}

	// The tracker reports signed pixel miss distance from the image center. Assume square pixels,
	// so the horizontal focal length in pixels is also used for the vertical miss conversion.
	los_x_rad = atan2f(static_cast<float>(miss_x), focal_length_px);
	los_y_rad = atan2f(static_cast<float>(miss_y), focal_length_px);
	return isfinite(los_x_rad) && isfinite(los_y_rad);
}

} // namespace dyt
