/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <math.h>
#include <stdint.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

class DytTerminalLosEstimator
{
public:
	struct Output {
		matrix::Vector3f los_raw{};
		matrix::Vector3f los_filtered{};
		matrix::Vector3f omega_raw{};
		matrix::Vector3f omega_filtered{};
		float los_step_rad{0.f};
		bool estimate_valid{false};
		bool rate_valid{false};
		bool rate_limited{false};
	};

	void reset()
	{
		_initialized = false;
		_sample_time = 0;
		_los_filtered.zero();
		_omega_filtered.zero();
		_output = Output{};
	}

	const Output &output() const { return _output; }

	bool update(const matrix::Vector3f &los_observed, uint64_t sample_time, float min_dt_s, float max_gap_s,
		    float los_time_constant_s, float omega_time_constant_s, float omega_max_rad_s)
	{
		_output.rate_valid = false;
		_output.rate_limited = false;
		_output.los_step_rad = 0.f;
		_output.omega_raw.zero();

		if (sample_time == 0 || !los_observed.isAllFinite() || los_observed.norm_squared() < 1e-6f) {
			_output.estimate_valid = false;
			return false;
		}

		matrix::Vector3f los_raw = los_observed.normalized();
		_output.los_raw = los_raw;

		if (!_initialized) {
			_initialized = true;
			_sample_time = sample_time;
			_los_filtered = los_raw;
			_omega_filtered.zero();
			_output.los_filtered = _los_filtered;
			_output.omega_filtered = _omega_filtered;
			_output.estimate_valid = true;
			return true;
		}

		if (sample_time <= _sample_time) {
			_output.estimate_valid = false;
			return false;
		}

		const float dt_s = static_cast<float>(sample_time - _sample_time) * 1e-6f;

		if (!PX4_ISFINITE(dt_s) || dt_s < min_dt_s) {
			_output.estimate_valid = false;
			return false;
		}

		if (dt_s > max_gap_s) {
			_sample_time = sample_time;
			_los_filtered = los_raw;
			_omega_filtered.zero();
			_output.los_filtered = _los_filtered;
			_output.omega_filtered = _omega_filtered;
			_output.estimate_valid = true;
			return true;
		}

		const float los_tc = math::max(los_time_constant_s, 0.001f);
		const float alpha = expf(-dt_s / los_tc);
		matrix::Vector3f filtered = _los_filtered * alpha + los_raw * (1.f - alpha);

		if (!filtered.isAllFinite() || filtered.norm_squared() < 1e-6f) {
			_output.estimate_valid = false;
			return false;
		}

		filtered.normalize();
		const matrix::Vector3f axis = _los_filtered.cross(filtered);
		const float axis_norm = axis.norm();
		const float dot = math::constrain(_los_filtered.dot(filtered), -1.f, 1.f);
		const float angle = atan2f(axis_norm, dot);
		matrix::Vector3f omega_raw{};

		if (axis_norm > 1e-6f) {
			omega_raw = axis * (angle / (axis_norm * dt_s));
		}

		_output.los_step_rad = angle;
		_output.omega_raw = omega_raw;
		const float omega_max = math::max(omega_max_rad_s, 0.01f);
		const bool rate_valid = omega_raw.isAllFinite() && omega_raw.norm() <= omega_max;

		if (!rate_valid) {
			// Preserve the last accepted LOS so one bad pixel sample cannot create a
			// second, opposite rate spike when the tracker returns to the target.
			_sample_time = sample_time;
			_output.rate_limited = true;
			_output.los_filtered = _los_filtered;
			_output.omega_filtered = _omega_filtered;
			_output.estimate_valid = true;
			return true;
		}

		const float omega_tc = math::max(omega_time_constant_s, 0.001f);
		const float beta = dt_s / (omega_tc + dt_s);
		_omega_filtered += (omega_raw - _omega_filtered) * beta;
		_los_filtered = filtered;
		_sample_time = sample_time;
		_output.los_filtered = _los_filtered;
		_output.omega_filtered = _omega_filtered;
		_output.estimate_valid = true;
		_output.rate_valid = true;
		return true;
	}

private:
	bool _initialized{false};
	uint64_t _sample_time{0};
	matrix::Vector3f _los_filtered{};
	matrix::Vector3f _omega_filtered{};
	Output _output{};
};

class DytTerminalVelocityGuidance
{
public:
	struct Output {
		matrix::Vector2f velocity{};
		matrix::Vector2f acceleration_raw{};
		matrix::Vector2f acceleration_limited{};
		bool acceleration_saturated{false};
		bool jerk_limited{false};
	};

	static Output update(const matrix::Vector2f &previous_velocity, const matrix::Vector2f &vehicle_velocity,
			     const matrix::Vector2f &previous_acceleration, const matrix::Vector2f &los_direction,
			     const matrix::Vector2f &turn_acceleration, const matrix::Vector2f &additional_acceleration,
			     float target_speed, float max_speed,
			     float max_acceleration, float max_jerk, float speed_slew, float dt_s)
	{
		Output output{};
		const float dt = math::constrain(dt_s, 0.005f, 0.1f);
		const float speed_limit = math::max(max_speed, 0.1f);
		const float desired_speed = math::constrain(target_speed, 0.f, speed_limit);
		matrix::Vector2f direction = previous_velocity;
		float previous_speed = direction.norm();

		if (!direction.isAllFinite() || previous_speed < 0.1f) {
			direction = vehicle_velocity;
			previous_speed = direction.norm();
		}

		if (!direction.isAllFinite() || previous_speed < 0.1f) {
			direction = los_direction;
			previous_speed = 0.f;
		}

		if (!direction.isAllFinite() || direction.norm() < 1e-3f) {
			return output;
		}

		direction.normalize();
		matrix::Vector2f acceleration = turn_acceleration;

		if (!acceleration.isAllFinite()) {
			acceleration.zero();
		}

		// LOS steering changes course only. Speed magnitude is handled by the
		// independent slew limiter below.
		acceleration -= direction * direction.dot(acceleration);

		if (additional_acceleration.isAllFinite()) {
			acceleration += additional_acceleration;
		}
		output.acceleration_raw = acceleration;
		const float acceleration_limit = math::max(max_acceleration, 0.1f);

		if (acceleration.norm() > acceleration_limit) {
			acceleration = acceleration.normalized() * acceleration_limit;
			output.acceleration_saturated = true;
		}

		matrix::Vector2f previous_accel = previous_acceleration;

		if (!previous_accel.isAllFinite()) {
			previous_accel.zero();
		}

		const matrix::Vector2f acceleration_delta = acceleration - previous_accel;
		const float max_acceleration_delta = math::max(max_jerk, 0.1f) * dt;

		if (acceleration_delta.norm() > max_acceleration_delta) {
			acceleration = previous_accel + acceleration_delta.normalized() * max_acceleration_delta;
			output.jerk_limited = true;
		}

		const matrix::Vector2f direction_velocity = direction * math::max(previous_speed, 0.1f) + acceleration * dt;

		if (direction_velocity.isAllFinite() && direction_velocity.norm() > 1e-3f) {
			direction = direction_velocity.normalized();
		}

		const float speed_delta = math::constrain(desired_speed - previous_speed,
					 -math::max(speed_slew, 0.1f) * dt, math::max(speed_slew, 0.1f) * dt);
		const float next_speed = math::constrain(previous_speed + speed_delta, 0.f, speed_limit);
		output.velocity = direction * next_speed;
		output.acceleration_limited = acceleration;
		return output;
	}
};
