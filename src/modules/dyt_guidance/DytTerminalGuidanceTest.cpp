/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "DytTerminalGuidance.hpp"

#include <gtest/gtest.h>
#include <string>

using matrix::Vector2f;
using matrix::Vector3f;

TEST(DytTerminalLosEstimator, ConstantLosHasZeroRate)
{
	DytTerminalLosEstimator estimator;
	ASSERT_TRUE(estimator.update(Vector3f(1.f, 0.f, 0.f), 1'000'000, 0.01f, 0.1f, 0.05f, 0.1f, 1.f));
	ASSERT_TRUE(estimator.update(Vector3f(1.f, 0.f, 0.f), 1'020'000, 0.01f, 0.1f, 0.05f, 0.1f, 1.f));
	EXPECT_TRUE(estimator.output().rate_valid);
	EXPECT_NEAR(estimator.output().omega_filtered.norm(), 0.f, 1e-6f);
}

TEST(DytTerminalLosEstimator, RecoversKnownAngularRateAcrossDt)
{
	DytTerminalLosEstimator estimator;
	ASSERT_TRUE(estimator.update(Vector3f(1.f, 0.f, 0.f), 1'000'000, 0.01f, 0.1f, 0.001f, 0.001f, 2.f));
	const float angle = 0.02f;
	ASSERT_TRUE(estimator.update(Vector3f(cosf(angle), sinf(angle), 0.f), 1'020'000,
				     0.01f, 0.1f, 0.001f, 0.001f, 2.f));
	EXPECT_TRUE(estimator.output().rate_valid);
	EXPECT_NEAR(estimator.output().omega_raw(2), 1.f, 0.01f);
	EXPECT_NEAR(estimator.output().omega_filtered(2), 1.f, 0.06f);
}

TEST(DytTerminalLosEstimator, RejectsSingleRateSpikeWithoutMovingLos)
{
	DytTerminalLosEstimator estimator;
	ASSERT_TRUE(estimator.update(Vector3f(1.f, 0.f, 0.f), 1'000'000, 0.01f, 0.1f, 0.001f, 0.1f, 1.f));
	ASSERT_TRUE(estimator.update(Vector3f(0.f, 1.f, 0.f), 1'020'000, 0.01f, 0.1f, 0.001f, 0.1f, 1.f));
	EXPECT_TRUE(estimator.output().rate_limited);
	EXPECT_FALSE(estimator.output().rate_valid);
	EXPECT_NEAR(estimator.output().los_filtered(0), 1.f, 1e-6f);
	EXPECT_NEAR(estimator.output().los_filtered(1), 0.f, 1e-6f);
	ASSERT_TRUE(estimator.update(Vector3f(1.f, 0.f, 0.f), 1'040'000, 0.01f, 0.1f, 0.001f, 0.1f, 1.f));
	EXPECT_TRUE(estimator.output().rate_valid);
	EXPECT_NEAR(estimator.output().omega_raw.norm(), 0.f, 1e-6f);
}

TEST(DytTerminalLosEstimator, ReinitializesAfterLongGapWithoutRateSpike)
{
	DytTerminalLosEstimator estimator;
	ASSERT_TRUE(estimator.update(Vector3f(1.f, 0.f, 0.f), 1'000'000, 0.01f, 0.12f, 0.08f, 0.15f, 0.8f));
	ASSERT_TRUE(estimator.update(Vector3f(0.98f, 0.2f, 0.f), 1'300'000, 0.01f, 0.12f, 0.08f, 0.15f, 0.8f));
	EXPECT_FALSE(estimator.output().rate_valid);
	EXPECT_NEAR(estimator.output().omega_filtered.norm(), 0.f, 1e-6f);
	EXPECT_NEAR(estimator.output().los_filtered.normalized().dot(Vector3f(0.98f, 0.2f, 0.f).normalized()),
		    1.f, 1e-6f);
}

TEST(DytTerminalLosEstimator, RejectsInvalidLos)
{
	DytTerminalLosEstimator estimator;
	EXPECT_FALSE(estimator.update(Vector3f(NAN, 0.f, 0.f), 1'000'000, 0.01f, 0.12f, 0.08f, 0.15f, 0.8f));
	EXPECT_FALSE(estimator.output().estimate_valid);
}

TEST(DytTerminalVelocityGuidance, LimitsAccelerationJerkAndSpeedSlew)
{
	const auto output = DytTerminalVelocityGuidance::update(
				Vector2f(45.f, 0.f), Vector2f(45.f, 0.f), Vector2f(0.f, 0.f), Vector2f(1.f, 0.f),
				Vector2f(0.f, 10.f), Vector2f{}, 50.f, 55.f, 4.f, 3.f, 2.f, 0.02f);
	EXPECT_TRUE(output.acceleration_saturated);
	EXPECT_TRUE(output.jerk_limited);
	EXPECT_LE(output.acceleration_limited.norm(), 0.060001f);
	EXPECT_NEAR(output.velocity.norm(), 45.04f, 0.001f);
}

TEST(DytTerminalVelocityGuidance, ReversesAccelerationContinuously)
{
	const auto output = DytTerminalVelocityGuidance::update(
				Vector2f(45.f, 0.f), Vector2f(45.f, 0.f), Vector2f(0.f, 2.f), Vector2f(1.f, 0.f),
				Vector2f(0.f, -4.f), Vector2f{}, 45.f, 50.f, 4.f, 4.f, 2.f, 0.1f);
	EXPECT_TRUE(output.jerk_limited);
	EXPECT_GT(output.acceleration_limited(1), 0.f);
	EXPECT_NEAR(output.acceleration_limited(1), 1.6f, 1e-5f);
}

TEST(DytTerminalVelocityGuidance, SlewsFromFortyToFiftyWithoutDirectionJump)
{
	Vector2f velocity(40.f, 0.f);
	Vector2f acceleration{};

	for (int sample = 0; sample < 250; ++sample) {
		const auto output = DytTerminalVelocityGuidance::update(
					velocity, velocity, acceleration, Vector2f(1.f, 0.f), Vector2f{}, Vector2f{},
					50.f, 50.f, 4.f, 4.f, 2.f, 0.02f);
		EXPECT_GE(output.velocity.norm(), velocity.norm());
		EXPECT_LE(output.velocity.norm() - velocity.norm(), 0.04001f);
		EXPECT_NEAR(output.velocity(1), 0.f, 1e-6f);
		velocity = output.velocity;
		acceleration = output.acceleration_limited;
	}

	EXPECT_NEAR(velocity.norm(), 50.f, 1e-3f);
}

TEST(DytTerminalVelocityGuidance, BoundsThreeHertzLosNoiseAtCruiseSpeed)
{
	constexpr float dt_s = 0.02f;
	constexpr float acceleration_limit = 4.f;
	constexpr float jerk_limit = 4.f;
	DytTerminalLosEstimator estimator;
	Vector2f velocity(45.f, 0.f);
	Vector2f acceleration{};
	float maximum_acceleration = 0.f;
	float maximum_acceleration_delta = 0.f;
	float maximum_course_error = 0.f;

	for (int sample = 0; sample < 500; ++sample) {
		const float time_s = sample * dt_s;
		const float los_y = 0.015f * sinf(2.f * M_PI_F * 3.f * time_s);
		Vector3f los_observed(1.f, los_y, 0.f);
		los_observed.normalize();
		ASSERT_TRUE(estimator.update(los_observed, 1'000'000 + sample * 20'000,
					     0.01f, 0.12f, 0.08f, 0.15f, 0.8f));

		const auto &estimate = estimator.output();
		Vector3f turn_acceleration = 1.2f * estimate.los_filtered;

		if (estimate.rate_valid) {
			turn_acceleration += 10.f * estimate.omega_filtered.cross(estimate.los_filtered);
		}

		const Vector2f previous_acceleration = acceleration;
		const auto output = DytTerminalVelocityGuidance::update(
					velocity, velocity, acceleration,
					Vector2f(estimate.los_filtered(0), estimate.los_filtered(1)),
					Vector2f(turn_acceleration(0), turn_acceleration(1)), Vector2f{},
					45.f, 50.f, acceleration_limit, jerk_limit, 2.f, dt_s);
		ASSERT_TRUE(output.velocity.isAllFinite());
		ASSERT_TRUE(output.acceleration_limited.isAllFinite());
		velocity = output.velocity;
		acceleration = output.acceleration_limited;
		maximum_acceleration = math::max(maximum_acceleration, acceleration.norm());
		maximum_acceleration_delta = math::max(maximum_acceleration_delta,
					      (acceleration - previous_acceleration).norm());
		maximum_course_error = math::max(maximum_course_error, fabsf(atan2f(velocity(1), velocity(0))));
	}

	EXPECT_LE(acceleration.norm(), acceleration_limit + 1e-5f);
	EXPECT_LE(maximum_acceleration_delta, jerk_limit * dt_s + 1e-5f);
	EXPECT_LT(maximum_course_error, math::radians(3.f));
	RecordProperty("maximum_acceleration_m_s2", std::to_string(maximum_acceleration));
	RecordProperty("maximum_acceleration_delta_m_s2", std::to_string(maximum_acceleration_delta));
	RecordProperty("maximum_course_error_deg", std::to_string(math::degrees(maximum_course_error)));
}
