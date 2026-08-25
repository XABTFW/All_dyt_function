#include "GnssEmergencyStateMachine.hpp"

#include <gtest/gtest.h>

TEST(GnssEmergencyStateMachine, AcceptsOneHertzLandStatusBetweenUpdates)
{
	EXPECT_FALSE(GnssEmergencyStateMachine::landedOrStatusUnavailable(2'500'000, 1'000'000, false));
	EXPECT_TRUE(GnssEmergencyStateMachine::landedOrStatusUnavailable(4'000'000, 1'000'000, false));
	EXPECT_TRUE(GnssEmergencyStateMachine::landedOrStatusUnavailable(1'000'000, 1'100'000, false));
	EXPECT_TRUE(GnssEmergencyStateMachine::landedOrStatusUnavailable(2'500'000, 1'000'000, true));
}

TEST(GnssEmergencyStateMachine, DescendsThenResumesAfterStableRecovery)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_active = true;
	input.interference = true;
	EXPECT_EQ(machine.update(1'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::Descend);

	input.mission_active = false;
	input.interference = false;
	input.navigation_recovered = true;
	EXPECT_EQ(machine.update(2'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.update(5'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::ResumeMission);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Idle);
}

TEST(GnssEmergencyStateMachine, RecoveryMustBeContinuous)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_active = true;
	input.interference = true;
	machine.update(1'000'000, input, 3'000'000);

	input.mission_active = false;
	input.interference = false;
	input.navigation_recovered = true;
	machine.update(2'000'000, input, 3'000'000);
	input.interference = true;
	input.navigation_recovered = false;
	EXPECT_EQ(machine.update(4'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::None);
	input.interference = false;
	input.navigation_recovered = true;
	machine.update(5'000'000, input, 3'000'000);
	EXPECT_EQ(machine.update(7'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.update(8'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::ResumeMission);
}

TEST(GnssEmergencyStateMachine, DoesNotResumeAfterTouchdown)
{
	GnssEmergencyStateMachine machine;
	GnssEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_active = true;
	input.interference = true;
	machine.update(1'000'000, input, 3'000'000);
	input.landed = true;
	input.interference = false;
	input.navigation_recovered = true;
	EXPECT_EQ(machine.update(2'000'000, input, 3'000'000), GnssEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), GnssEmergencyStateMachine::State::Idle);
}
