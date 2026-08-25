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

#include "FastTouchdownDetector.hpp"

#include <px4_platform_common/defines.h>

namespace land_detector
{

bool FastTouchdownDetector::update(hrt_abstime now, bool enabled, bool armed,
				   bool landing_allowed,
				   bool touchdown_allowed,
				   float height_above_home,
				   float maximum_height_above_home,
				   const distance_sensor_s *sample,
				   float trigger_distance,
				   float operational_max_distance,
				   hrt_abstime trigger_time)
{
	if (!enabled || !armed || !PX4_ISFINITE(trigger_distance) || trigger_distance <= 0.f
	    || !PX4_ISFINITE(operational_max_distance)
	    || operational_max_distance <= trigger_distance
	    || !PX4_ISFINITE(maximum_height_above_home) || maximum_height_above_home <= 0.f) {
		reset();
		return false;
	}

	// Once touchdown is confirmed, keep it latched until disarm, disable, or invalid parameters reset it.
	if (_triggered) {
		return true;
	}

	const bool independent_altitude_gate = PX4_ISFINITE(height_above_home)
					       && height_above_home >= -HEIGHT_BELOW_HOME_TOLERANCE
					       && height_above_home <= maximum_height_above_home;

	if (!landing_allowed || !independent_altitude_gate) {
		reset();
		return false;
	}

	if (sample == nullptr) {
		if (_last_sample_time != 0 &&
		    (now < _last_sample_time || now - _last_sample_time > SAMPLE_TIMEOUT)) {
			resetSampleHistory();
		}

		return false;
	}

	if (!validSample(*sample, now, operational_max_distance)) {
		resetSampleHistory();
		return false;
	}

	if (sample->timestamp <= _last_sample_time) {
		return false;
	}

	if (_last_sample_time != 0) {
		const hrt_abstime sample_interval = sample->timestamp - _last_sample_time;

		if (sample_interval > MAX_SAMPLE_GAP) {
			resetSampleHistory();

		} else {
			const float rate_limited_drop = MAX_DISTANCE_DROP_RATE * sample_interval * 1e-6f + DISTANCE_JUMP_MARGIN;
			const float maximum_distance_drop = rate_limited_drop < MAX_DISTANCE_DROP ? rate_limited_drop : MAX_DISTANCE_DROP;

			if (_last_sample_distance - sample->current_distance > maximum_distance_drop) {
				resetSampleHistory();
			}
		}
	}

	_last_sample_time = sample->timestamp;
	_last_sample_distance = sample->current_distance;

	if (sample->current_distance > trigger_distance) {
		resetCandidate();
		return false;
	}

	if (!touchdown_allowed) {
		resetCandidate();
		return false;
	}

	if (_sample_count == 0) {
		_first_sample_time = sample->timestamp;
	}

	++_sample_count;

	if (_sample_count >= MIN_SAMPLE_COUNT &&
	    sample->timestamp - _first_sample_time >= trigger_time) {
		_triggered = true;
		_trigger_distance = sample->current_distance;
		_confirmation_time = sample->timestamp - _first_sample_time;
	}

	return _triggered;
}

bool FastTouchdownDetector::validSample(const distance_sensor_s &sample,
					hrt_abstime now,
					float operational_max_distance) const
{
	return sample.timestamp > 0 && sample.timestamp <= now &&
	       now - sample.timestamp <= SAMPLE_TIMEOUT &&
	       sample.type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
	       sample.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
	       sample.signal_quality != 0 && PX4_ISFINITE(sample.current_distance) &&
	       PX4_ISFINITE(sample.min_distance) &&
	       PX4_ISFINITE(sample.max_distance) &&
	       sample.min_distance <= sample.max_distance &&
	       sample.current_distance >= sample.min_distance &&
	       sample.current_distance <= sample.max_distance &&
	       sample.current_distance <= operational_max_distance;
}

void FastTouchdownDetector::resetCandidate()
{
	_first_sample_time = 0;
	_sample_count = 0;
}

void FastTouchdownDetector::resetSampleHistory()
{
	resetCandidate();
	_last_sample_time = 0;
	_last_sample_distance = 0.f;
}

void FastTouchdownDetector::reset()
{
	resetSampleHistory();
	_confirmation_time = 0;
	_trigger_distance = 0.f;
	_triggered = false;
}

} // namespace land_detector
