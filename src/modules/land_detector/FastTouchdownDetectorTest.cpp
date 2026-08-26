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

#include <gtest/gtest.h>

#include <math.h>

#include "FastTouchdownDetector.hpp"

using land_detector::FastTouchdownDetector;
using namespace time_literals;

class FastTouchdownDetectorTest : public ::testing::Test
{
protected:
	distance_sensor_s sample(hrt_abstime timestamp, float distance = 0.2f)
	{
		distance_sensor_s result{};
		result.timestamp = timestamp;
		result.min_distance = 0.02f;
		result.max_distance = 2.f;
		result.current_distance = distance;
		result.signal_quality = -1;
		result.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		result.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		return result;
	}

	bool update(const distance_sensor_s *sample, hrt_abstime now,
		    bool enabled = true, bool armed = true,
		    bool landing_allowed = true, bool touchdown_allowed = true,
		    float height_above_home = 1.f, float maximum_height_above_home = 5.f,
		    bool tof_touchdown_allowed = true)
	{
		return _detector.update(now, enabled, armed, landing_allowed, touchdown_allowed, tof_touchdown_allowed,
					height_above_home, maximum_height_above_home, 0.5f,
					sample, 0.28f, 5.f, 40_ms);
	}

	hrt_abstime feedApproach(hrt_abstime start = 900_ms)
	{
		static constexpr float distances[] {0.8f, 0.7f, 0.6f, 0.55f, 0.45f, 0.35f, 0.29f};
		hrt_abstime timestamp = start;

		for (float distance : distances) {
			auto reading = sample(timestamp, distance);
			EXPECT_FALSE(update(&reading, reading.timestamp));
			timestamp += 20_ms;
		}

		return timestamp - 20_ms;
	}

	FastTouchdownDetector _detector{};
};

TEST_F(FastTouchdownDetectorTest, RequiresThreeSamplesAndConfirmationTime)
{
	const hrt_abstime approach_end = feedApproach();
	auto first = sample(approach_end + 20_ms);
	auto second = sample(approach_end + 40_ms);
	auto third = sample(approach_end + 60_ms);

	EXPECT_FALSE(update(&first, first.timestamp));
	EXPECT_FALSE(update(&second, second.timestamp));
	EXPECT_TRUE(update(&third, third.timestamp));
	EXPECT_TRUE(_detector.triggered());
	EXPECT_EQ(_detector.confirmationTime(), 40_ms);
}

TEST_F(FastTouchdownDetectorTest, RejectsDisabledDisarmedAndWrongMode)
{
	auto reading = sample(1_s);

	EXPECT_FALSE(update(&reading, reading.timestamp, false));
	EXPECT_FALSE(update(&reading, reading.timestamp, true, false));
	EXPECT_FALSE(update(&reading, reading.timestamp, true, true, false));
}

TEST_F(FastTouchdownDetectorTest, RejectsInvalidSensorData)
{
	auto reading = sample(1_s);
	reading.signal_quality = 0;
	EXPECT_FALSE(update(&reading, reading.timestamp));

	reading = sample(1020_ms);
	reading.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;
	EXPECT_FALSE(update(&reading, reading.timestamp));

	reading = sample(1040_ms);
	reading.current_distance = NAN;
	EXPECT_FALSE(update(&reading, reading.timestamp));

	reading = sample(1060_ms, 3.f);
	EXPECT_FALSE(update(&reading, reading.timestamp));
}

TEST_F(FastTouchdownDetectorTest, RejectsInvalidTriggerDistance)
{
	auto reading = sample(1_s);
	EXPECT_FALSE(_detector.update(reading.timestamp, true, true, true, true, true, 1.f, 5.f, 0.5f, &reading, NAN, 5.f,
				      40_ms));
	EXPECT_FALSE(_detector.update(reading.timestamp, true, true, true, true, true, 1.f, 5.f, 0.5f, &reading, 0.f, 5.f,
				      40_ms));
	EXPECT_FALSE(_detector.update(reading.timestamp, true, true, true, true, true, 1.f, 5.f, 0.5f, &reading, 0.28f, 0.28f,
				      40_ms));
	EXPECT_FALSE(_detector.update(reading.timestamp, true, true, true, true, true, 1.f, NAN, 0.5f, &reading, 0.28f, 5.f,
				      40_ms));
}

TEST_F(FastTouchdownDetectorTest, AboveThresholdResetsCandidate)
{
	const hrt_abstime approach_end = feedApproach();
	auto first = sample(approach_end + 20_ms);
	auto high = sample(approach_end + 40_ms, 0.29f);
	auto second = sample(approach_end + 60_ms);
	auto third = sample(approach_end + 80_ms);
	auto trigger = sample(approach_end + 100_ms);

	EXPECT_FALSE(update(&first, first.timestamp));
	EXPECT_FALSE(update(&high, high.timestamp));
	EXPECT_FALSE(update(&second, second.timestamp));
	EXPECT_FALSE(update(&third, third.timestamp));
	EXPECT_TRUE(update(&trigger, trigger.timestamp));
}

TEST_F(FastTouchdownDetectorTest,
	StaleSampleResetsAndTriggerLatchesUntilDisarmed)
{
	const hrt_abstime approach_end = feedApproach();
	auto first = sample(approach_end + 20_ms);
	auto second = sample(approach_end + 300_ms);
	auto third = sample(approach_end + 320_ms);

	EXPECT_FALSE(update(&first, first.timestamp));
	EXPECT_FALSE(update(nullptr, first.timestamp + 251_ms));
	EXPECT_FALSE(update(&second, second.timestamp));
	EXPECT_FALSE(update(&third, third.timestamp));

	const hrt_abstime second_approach_end = feedApproach(approach_end + 400_ms);
	auto fourth = sample(second_approach_end + 20_ms);
	auto fifth = sample(second_approach_end + 40_ms);
	auto sixth = sample(second_approach_end + 60_ms);
	EXPECT_FALSE(update(&fourth, fourth.timestamp));
	EXPECT_FALSE(update(&fifth, fifth.timestamp));
	EXPECT_TRUE(update(&sixth, sixth.timestamp));
	EXPECT_TRUE(update(nullptr, 2_s, true, true, false));
	EXPECT_TRUE(_detector.triggered());
	EXPECT_FALSE(update(nullptr, 2_s, true, false, false));
	EXPECT_FALSE(_detector.triggered());
}

TEST_F(FastTouchdownDetectorTest, LowAtLandingEntryTriggersAfterConfirmation)
{
	auto first = sample(1_s, 0.04f);
	auto second = sample(1020_ms, 0.04f);
	auto third = sample(1040_ms, 0.04f);

	EXPECT_FALSE(update(&first, first.timestamp));
	EXPECT_FALSE(update(&second, second.timestamp));
	EXPECT_TRUE(update(&third, third.timestamp));
}

TEST_F(FastTouchdownDetectorTest, ConfirmedTwoMetreApproachDoesNotRequireEkfGroundDistance)
{
	auto approach_first = sample(900_ms, 1.0f);
	auto approach_second = sample(920_ms, 0.8f);
	auto approach_third = sample(940_ms, 0.6f);
	auto approach_fourth = sample(960_ms, 0.45f);
	auto approach_fifth = sample(980_ms, 0.3f);
	auto approach_sixth = sample(1_s, 0.29f);
	auto first = sample(1040_ms, 0.04f);
	auto second = sample(1060_ms, 0.04f);
	auto third = sample(1080_ms, 0.04f);

	EXPECT_FALSE(update(&approach_first, approach_first.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&approach_second, approach_second.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&approach_third, approach_third.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&approach_fourth, approach_fourth.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&approach_fifth, approach_fifth.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&approach_sixth, approach_sixth.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&first, first.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&second, second.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_TRUE(update(&third, third.timestamp, true, true, true, false, 1.f, 5.f, true));
}

TEST_F(FastTouchdownDetectorTest, SuddenLowReadingDoesNotBypassEkfGroundDistance)
{
	auto first = sample(1_s, 0.04f);
	auto second = sample(1020_ms, 0.04f);
	auto third = sample(1040_ms, 0.04f);

	EXPECT_FALSE(update(&first, first.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&second, second.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(update(&third, third.timestamp, true, true, true, false, 1.f, 5.f, true));
	EXPECT_FALSE(_detector.triggered());
}

TEST_F(FastTouchdownDetectorTest, ConfirmedApproachStillRequiresNonEkfSafetyGates)
{
	const hrt_abstime approach_end = feedApproach();
	auto first = sample(approach_end + 20_ms);
	auto second = sample(approach_end + 40_ms);
	auto third = sample(approach_end + 60_ms);

	EXPECT_FALSE(update(&first, first.timestamp, true, true, true, false, 1.f, 5.f, false));
	EXPECT_FALSE(update(&second, second.timestamp, true, true, true, false, 1.f, 5.f, false));
	EXPECT_FALSE(update(&third, third.timestamp, true, true, true, false, 1.f, 5.f, false));
	EXPECT_FALSE(_detector.triggered());
}

TEST_F(FastTouchdownDetectorTest, RejectsOutsideOperationalRangeAndLargeDistanceJump)
{
	auto out_of_range = sample(1_s, 5.1f);
	out_of_range.max_distance = 6.f;
	EXPECT_FALSE(update(&out_of_range, out_of_range.timestamp));

	auto low_after_invalid = sample(1020_ms, 0.04f);
	EXPECT_FALSE(update(&low_after_invalid, low_after_invalid.timestamp));

	auto approach = sample(1040_ms, 4.f);
	approach.max_distance = 6.f;
	EXPECT_FALSE(update(&approach, approach.timestamp));

	auto implausible_low = sample(1060_ms, 0.04f);
	auto stable_low = sample(1080_ms, 0.04f);
	auto confirmed_low = sample(1100_ms, 0.04f);
	EXPECT_FALSE(update(&implausible_low, implausible_low.timestamp));
	EXPECT_FALSE(update(&stable_low, stable_low.timestamp));
	EXPECT_TRUE(update(&confirmed_low, confirmed_low.timestamp));
}

TEST_F(FastTouchdownDetectorTest, AcceptsSlowerContinuousMeasurements)
{
	static constexpr float approach_distances[] {0.8f, 0.65f, 0.55f, 0.4f, 0.29f};
	hrt_abstime timestamp = 1_s;

	for (float distance : approach_distances) {
		auto reading = sample(timestamp, distance);
		EXPECT_FALSE(update(&reading, reading.timestamp));
		timestamp += 200_ms;
	}

	auto first = sample(timestamp);
	auto second = sample(timestamp + 200_ms);
	auto third = sample(timestamp + 400_ms);
	EXPECT_FALSE(update(&first, first.timestamp));
	EXPECT_FALSE(update(&second, second.timestamp));
	EXPECT_TRUE(update(&third, third.timestamp));
}

TEST_F(FastTouchdownDetectorTest, LargeJumpRestartsConfirmation)
{
	auto approach = sample(1_s, 1.23f);
	auto implausible_low = sample(1250_ms, 0.02f);
	auto second_low = sample(1500_ms, 0.02f);
	auto third_low = sample(1750_ms, 0.02f);

	EXPECT_FALSE(update(&approach, approach.timestamp));
	EXPECT_FALSE(update(&implausible_low, implausible_low.timestamp));
	EXPECT_FALSE(update(&second_low, second_low.timestamp));
	EXPECT_TRUE(update(&third_low, third_low.timestamp));
}

TEST_F(FastTouchdownDetectorTest, AcceptsDropBelowAbsoluteLimit)
{
	auto approach = sample(1_s, 1.5f);
	auto lower = sample(1250_ms, 0.31f);
	auto first = sample(1500_ms);
	auto second = sample(1750_ms);
	auto third = sample(2_s);

	EXPECT_FALSE(update(&approach, approach.timestamp));
	EXPECT_FALSE(update(&lower, lower.timestamp));
	EXPECT_FALSE(update(&first, first.timestamp));
	EXPECT_FALSE(update(&second, second.timestamp));
	EXPECT_TRUE(update(&third, third.timestamp));
}

TEST_F(FastTouchdownDetectorTest, EkfTouchdownGateRequiredWithoutConfirmedApproach)
{
	auto first = sample(1_s, 0.04f);
	auto second = sample(1020_ms, 0.04f);
	auto third = sample(1040_ms, 0.04f);

	EXPECT_FALSE(update(&first, first.timestamp, true, true, true, false));
	EXPECT_FALSE(update(&second, second.timestamp, true, true, true, false));
	EXPECT_FALSE(update(&third, third.timestamp, true, true, true, false));

	auto fourth = sample(1060_ms, 0.04f);
	auto fifth = sample(1080_ms, 0.04f);
	auto sixth = sample(1100_ms, 0.04f);
	EXPECT_FALSE(update(&fourth, fourth.timestamp));
	EXPECT_FALSE(update(&fifth, fifth.timestamp));
	EXPECT_TRUE(update(&sixth, sixth.timestamp));
}

TEST_F(FastTouchdownDetectorTest, IndependentAltitudeGateMustPass)
{
	auto first_high = sample(1_s, 0.04f);
	auto second_high = sample(1020_ms, 0.04f);
	auto third_high = sample(1040_ms, 0.04f);

	EXPECT_FALSE(update(&first_high, first_high.timestamp, true, true, true, true, 5.1f));
	EXPECT_FALSE(update(&second_high, second_high.timestamp, true, true, true, true, 5.1f));
	EXPECT_FALSE(update(&third_high, third_high.timestamp, true, true, true, true, 5.1f));
	EXPECT_FALSE(_detector.triggered());

	auto first_low = sample(1060_ms, 0.04f);
	auto second_low = sample(1080_ms, 0.04f);
	auto third_low = sample(1100_ms, 0.04f);

	EXPECT_FALSE(update(&first_low, first_low.timestamp, true, true, true, true, 5.f));
	EXPECT_FALSE(update(&second_low, second_low.timestamp, true, true, true, true, 5.f));
	EXPECT_TRUE(update(&third_low, third_low.timestamp, true, true, true, true, 5.f));
}

TEST_F(FastTouchdownDetectorTest, IndependentAltitudeGateFailsClosed)
{
	auto first = sample(1_s, 0.04f);
	auto second = sample(1020_ms, 0.04f);
	auto third = sample(1040_ms, 0.04f);

	EXPECT_FALSE(update(&first, first.timestamp, true, true, true, true, NAN));
	EXPECT_FALSE(update(&second, second.timestamp, true, true, true, true, -0.6f));
	EXPECT_FALSE(update(&third, third.timestamp, true, true, true, true, 1.f, 0.f));
	EXPECT_FALSE(_detector.triggered());
}

TEST_F(FastTouchdownDetectorTest, ConfigurableBelowHomeTolerance)
{
	auto first = sample(1_s, 0.04f);
	auto second = sample(1020_ms, 0.04f);
	auto third = sample(1040_ms, 0.04f);

	EXPECT_FALSE(_detector.update(first.timestamp, true, true, true, true, true, -5.3f, 5.f, 6.f,
				      &first, 0.28f, 5.f, 40_ms));
	EXPECT_FALSE(_detector.update(second.timestamp, true, true, true, true, true, -5.3f, 5.f, 6.f,
				      &second, 0.28f, 5.f, 40_ms));
	EXPECT_TRUE(_detector.update(third.timestamp, true, true, true, true, true, -5.3f, 5.f, 6.f,
				     &third, 0.28f, 5.f, 40_ms));
}
