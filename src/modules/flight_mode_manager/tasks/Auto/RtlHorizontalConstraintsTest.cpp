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

#include "RtlHorizontalConstraints.hpp"

#include <gtest/gtest.h>

using namespace rtl_constraints;

TEST(RtlHorizontalConstraintsTest, ActivationAltitudeLatchesUntilRtlEnds)
{
	EXPECT_FALSE(updateRtlHorizontalConstraintsActivation(true, false, 10.f, 5.f));
	EXPECT_TRUE(updateRtlHorizontalConstraintsActivation(true, false, 5.f, 5.f));
	EXPECT_TRUE(updateRtlHorizontalConstraintsActivation(true, true, 6.f, 5.f));
	EXPECT_FALSE(updateRtlHorizontalConstraintsActivation(false, true, 2.f, 5.f));
}

TEST(RtlHorizontalConstraintsTest, NonPositiveActivationAltitudeKeepsPreviousBehavior)
{
	EXPECT_TRUE(updateRtlHorizontalConstraintsActivation(true, false, 50.f, -1.f));
}

TEST(RtlHorizontalConstraintsTest, InvalidAltitudeDoesNotActivateLowAltitudeConstraints)
{
	EXPECT_FALSE(updateRtlHorizontalConstraintsActivation(true, false, NAN, 5.f));
}

TEST(RtlHorizontalConstraintsTest, NonRtlKeepsBaseConstraints)
{
	const auto constraints = selectRtlHorizontalConstraints(false, 8.f, 12.f, 3.f, 4.f, 5.f, 1.5f, 2.f);

	EXPECT_FLOAT_EQ(constraints.cruise_speed, 8.f);
	EXPECT_FLOAT_EQ(constraints.acceleration, 3.f);
	EXPECT_FLOAT_EQ(constraints.jerk, 4.f);
}

TEST(RtlHorizontalConstraintsTest, DisabledOverridesKeepBaseConstraints)
{
	const auto constraints = selectRtlHorizontalConstraints(true, 8.f, 12.f, 3.f, 4.f, -1.f, -1.f, -1.f);

	EXPECT_FLOAT_EQ(constraints.cruise_speed, 8.f);
	EXPECT_FLOAT_EQ(constraints.acceleration, 3.f);
	EXPECT_FLOAT_EQ(constraints.jerk, 4.f);
}

TEST(RtlHorizontalConstraintsTest, PositiveValuesOverrideIndependently)
{
	auto constraints = selectRtlHorizontalConstraints(true, 8.f, 12.f, 3.f, 4.f, 5.f, -1.f, -1.f);
	EXPECT_FLOAT_EQ(constraints.cruise_speed, 5.f);
	EXPECT_FLOAT_EQ(constraints.acceleration, 3.f);
	EXPECT_FLOAT_EQ(constraints.jerk, 4.f);

	constraints = selectRtlHorizontalConstraints(true, 8.f, 12.f, 3.f, 4.f, -1.f, 1.5f, -1.f);
	EXPECT_FLOAT_EQ(constraints.cruise_speed, 8.f);
	EXPECT_FLOAT_EQ(constraints.acceleration, 1.5f);
	EXPECT_FLOAT_EQ(constraints.jerk, 4.f);

	constraints = selectRtlHorizontalConstraints(true, 8.f, 12.f, 3.f, 4.f, -1.f, -1.f, 2.f);
	EXPECT_FLOAT_EQ(constraints.cruise_speed, 8.f);
	EXPECT_FLOAT_EQ(constraints.acceleration, 3.f);
	EXPECT_FLOAT_EQ(constraints.jerk, 2.f);
}

TEST(RtlHorizontalConstraintsTest, RtlSpeedIsLimitedByGlobalMaximum)
{
	const auto constraints = selectRtlHorizontalConstraints(true, 8.f, 12.f, 3.f, 4.f, 20.f, -1.f, -1.f);

	EXPECT_FLOAT_EQ(constraints.cruise_speed, 12.f);
}

TEST(RtlHorizontalConstraintsTest, InvalidOverridesKeepBaseConstraints)
{
	const float nan = NAN;
	const auto constraints = selectRtlHorizontalConstraints(true, 8.f, 12.f, 3.f, 4.f, nan, nan, nan);

	EXPECT_FLOAT_EQ(constraints.cruise_speed, 8.f);
	EXPECT_FLOAT_EQ(constraints.acceleration, 3.f);
	EXPECT_FLOAT_EQ(constraints.jerk, 4.f);
}
