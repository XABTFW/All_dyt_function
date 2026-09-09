#include "GnssEmergencyStateMachine.hpp"

#include <gtest/gtest.h>

TEST(GnssEmergencyStateMachine, AcceptsOneHertzLandStatusBetweenUpdates)
{
	EXPECT_FALSE(GnssEmergencyStateMachine::landedOrStatusUnavailable(2'500'000, 1'000'000, false));
	EXPECT_TRUE(GnssEmergencyStateMachine::landedOrStatusUnavailable(4'000'000, 1'000'000, false));
	EXPECT_TRUE(GnssEmergencyStateMachine::landedOrStatusUnavailable(1'000'000, 1'100'000, false));
	EXPECT_TRUE(GnssEmergencyStateMachine::landedOrStatusUnavailable(2'500'000, 1'000'000, true));
}

TEST(GnssEmergencyStateMachine, LandsInAnyAirborneMode)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.gnss_failure = true;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::Land);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Landing);
}

TEST(GnssEmergencyStateMachine, GnssRecoveryAloneDoesNotInterruptLanding)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.gnss_failure = true;
	machine.update(input);

	input.gnss_failure = false;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Landing);
}

TEST(GnssEmergencyStateMachine, RcTakeoverReleasesLandingUntilGnssRecovers)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.gnss_failure = true;
	machine.update(input);

	input.manual_control_available = true;
	input.manual_takeover = true;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Released);

	input.manual_takeover = false;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Released);

	input.gnss_failure = false;
	machine.update(input);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Idle);

	input.gnss_failure = true;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::Land);
}

TEST(GnssEmergencyStateMachine, RcLossAfterTakeoverRestartsLanding)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.gnss_failure = true;
	machine.update(input);

	input.manual_control_available = true;
	input.manual_takeover = true;
	machine.update(input);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Released);

	input.manual_control_available = false;
	input.manual_takeover = false;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::Land);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Landing);
}

TEST(GnssEmergencyStateMachine, LandingOrDisarmingClearsEmergency)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.gnss_failure = true;
	machine.update(input);

	input.landed = true;
	EXPECT_EQ(machine.update(input), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Idle);
}
