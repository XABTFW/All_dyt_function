#include "CommEmergencyStateMachine.hpp"

#include <gtest/gtest.h>

TEST(CommEmergencyStateMachine, LandStatusAllowsOneHertzPublicationJitter)
{
	EXPECT_FALSE(CommEmergencyStateMachine::landedOrStatusUnavailable(2'500'000, 1'000'000, false));
	EXPECT_FALSE(CommEmergencyStateMachine::landedOrStatusUnavailable(3'999'999, 1'000'000, false));
	EXPECT_TRUE(CommEmergencyStateMachine::landedOrStatusUnavailable(4'000'000, 1'000'000, false));
	EXPECT_TRUE(CommEmergencyStateMachine::landedOrStatusUnavailable(2'000'000, 0, false));
	EXPECT_TRUE(CommEmergencyStateMachine::landedOrStatusUnavailable(2'000'000, 2'000'001, false));
	EXPECT_TRUE(CommEmergencyStateMachine::landedOrStatusUnavailable(2'000'000, 1'000'000, true));
}

TEST(CommEmergencyStateMachine, RecoversMissionDuringHold)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	EXPECT_EQ(machine.update(1'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Hold);

	input.mission_intended = false;
	input.link_lost = false;
	EXPECT_EQ(machine.update(20'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::ResumeMission);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Idle);
}

TEST(CommEmergencyStateMachine, ReturnsAfterTimeout)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	input.mission_intended = false;
	EXPECT_EQ(machine.update(31'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Return);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Committed);
}

TEST(CommEmergencyStateMachine, LowBatteryLandsWhenReturnIsNotFeasible)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	input.mission_intended = false;
	input.battery_below_threshold = true;
	input.rtl_feasible = false;
	EXPECT_EQ(machine.update(2'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Land);
}

TEST(CommEmergencyStateMachine, LowBatteryReturnsWhenReturnIsFeasible)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	input.mission_intended = false;
	input.battery_below_threshold = true;
	input.rtl_feasible = true;
	EXPECT_EQ(machine.update(2'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Return);
}

TEST(CommEmergencyStateMachine, LowBatteryHasPriorityOverLinkRecovery)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	input.mission_intended = false;
	input.link_lost = false;
	input.battery_below_threshold = true;
	input.rtl_feasible = false;
	EXPECT_EQ(machine.update(2'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Land);
}

TEST(CommEmergencyStateMachine, ManualFlightHoldsThenReturnsAfterTimeout)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = false;
	input.link_lost = true;
	EXPECT_EQ(machine.update(1'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Hold);
	EXPECT_EQ(machine.update(31'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Return);
}

TEST(CommEmergencyStateMachine, ManualFlightDoesNotStartMissionAfterRecovery)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = false;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);

	input.link_lost = false;
	EXPECT_EQ(machine.update(2'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Idle);
}

TEST(CommEmergencyStateMachine, LandsAfterTimeoutWhenConfigured)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000, CommEmergencyStateMachine::Action::Land);
	EXPECT_EQ(machine.update(31'000'000, input, 30'000'000, CommEmergencyStateMachine::Action::Land),
		  CommEmergencyStateMachine::Action::Land);
}
