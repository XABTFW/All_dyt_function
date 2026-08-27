#pragma once

#include <float.h>
#include <math.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/home_position.h>

namespace home_position_utils
{

inline float selectManualAltitude(float requested_altitude, const home_position_s &current_home,
				  float maximum_altitude_delta, bool &limited)
{
	limited = false;

	if (!current_home.valid_alt || !PX4_ISFINITE(current_home.alt)) {
		return requested_altitude;
	}

	const float altitude_delta_limit = PX4_ISFINITE(maximum_altitude_delta) ?
					   math::constrain(maximum_altitude_delta, 0.f, 1000.f) : 30.f;

	if (altitude_delta_limit <= FLT_EPSILON
	    || fabsf(requested_altitude - current_home.alt) > altitude_delta_limit) {
		limited = true;
		return current_home.alt;
	}

	return requested_altitude;
}

} // namespace home_position_utils
