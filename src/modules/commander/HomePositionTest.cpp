#include <gtest/gtest.h>

#include "HomePositionAltitudeGuard.hpp"

TEST(HomePositionTest, AcceptsPlausibleManualAltitude)
{
	home_position_s home{};
	home.valid_alt = true;
	home.alt = 100.f;
	bool limited = false;

	EXPECT_FLOAT_EQ(home_position_utils::selectManualAltitude(120.f, home, 30.f, limited), 120.f);
	EXPECT_FALSE(limited);
}

TEST(HomePositionTest, RetainsExistingAltitudeForImplausibleManualAltitude)
{
	home_position_s home{};
	home.valid_alt = true;
	home.alt = 100.f;
	bool limited = false;

	EXPECT_FLOAT_EQ(home_position_utils::selectManualAltitude(500.f, home, 30.f, limited), 100.f);
	EXPECT_TRUE(limited);
}

TEST(HomePositionTest, RetainsExistingAltitudeForLargeNegativeDelta)
{
	home_position_s home{};
	home.valid_alt = true;
	home.alt = 500.f;
	bool limited = false;

	EXPECT_FLOAT_EQ(home_position_utils::selectManualAltitude(100.f, home, 30.f, limited), 500.f);
	EXPECT_TRUE(limited);
}

TEST(HomePositionTest, ZeroLimitAlwaysRetainsExistingAltitude)
{
	home_position_s home{};
	home.valid_alt = true;
	home.alt = 100.f;
	bool limited = false;

	EXPECT_FLOAT_EQ(home_position_utils::selectManualAltitude(101.f, home, 0.f, limited), 100.f);
	EXPECT_TRUE(limited);
}

TEST(HomePositionTest, AcceptsRequestedAltitudeWithoutExistingHomeAltitude)
{
	home_position_s home{};
	bool limited = false;

	EXPECT_FLOAT_EQ(home_position_utils::selectManualAltitude(500.f, home, 30.f, limited), 500.f);
	EXPECT_FALSE(limited);
}
