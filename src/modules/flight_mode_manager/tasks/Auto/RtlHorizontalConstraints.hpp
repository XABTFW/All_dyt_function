/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

namespace rtl_constraints
{

struct RtlHorizontalConstraints {
	float cruise_speed;
	float acceleration;
	float jerk;
};

inline bool updateRtlHorizontalConstraintsActivation(bool rtl_rotary_wing, bool already_active,
		float distance_to_ground, float activation_altitude)
{
	if (!rtl_rotary_wing) {
		return false;
	}

	if (already_active || (PX4_ISFINITE(activation_altitude) && activation_altitude <= 0.f)) {
		return true;
	}

	return PX4_ISFINITE(distance_to_ground) && PX4_ISFINITE(activation_altitude)
	       && distance_to_ground <= activation_altitude;
}

inline RtlHorizontalConstraints selectRtlHorizontalConstraints(bool rtl_rotary_wing,
		float base_cruise_speed, float max_horizontal_speed,
		float base_acceleration, float base_jerk,
		float rtl_cruise_speed, float rtl_acceleration, float rtl_jerk)
{
	RtlHorizontalConstraints constraints{base_cruise_speed, base_acceleration, base_jerk};

	if (!rtl_rotary_wing) {
		return constraints;
	}

	if (PX4_ISFINITE(rtl_cruise_speed) && rtl_cruise_speed > 0.f) {
		constraints.cruise_speed = math::min(rtl_cruise_speed, max_horizontal_speed);
	}

	if (PX4_ISFINITE(rtl_acceleration) && rtl_acceleration > 0.f) {
		constraints.acceleration = rtl_acceleration;
	}

	if (PX4_ISFINITE(rtl_jerk) && rtl_jerk > 0.f) {
		constraints.jerk = rtl_jerk;
	}

	return constraints;
}

} // namespace rtl_constraints
