#include "CommEmergencyStateMachine.hpp"
#include "FlightDistanceTracker.hpp"

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
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Resuming);
	machine.resumeCompleted();
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Idle);
}

TEST(CommEmergencyStateMachine, NormalReturnWithHealthyLinkIsUnaffected)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.return_active = true;
	input.resume_distance_allowed = true;

	EXPECT_EQ(machine.update(1'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Idle);
}

TEST(CommEmergencyStateMachine, RecoversOffboardDuringHold)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.offboard_intended = true;
	input.link_lost = true;
	EXPECT_EQ(machine.update(1'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Hold);

	input.offboard_intended = false;
	input.link_lost = false;
	EXPECT_EQ(machine.update(2'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::ResumeOffboard);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Resuming);
}

TEST(CommEmergencyStateMachine, RecoversMissionFromLinkLossReturnBelowDistanceLimit)
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

	input.link_lost = false;
	input.return_active = true;
	input.resume_distance_allowed = true;
	EXPECT_EQ(machine.update(32'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::ResumeMission);
	EXPECT_TRUE(machine.resumeFromReturn());
}

TEST(CommEmergencyStateMachine, RecoversOffboardFromLinkLossReturnBelowDistanceLimit)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.offboard_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	input.offboard_intended = false;
	machine.update(31'000'000, input, 30'000'000);

	input.link_lost = false;
	input.return_active = true;
	input.resume_distance_allowed = true;
	EXPECT_EQ(machine.update(32'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::ResumeOffboard);
	EXPECT_TRUE(machine.resumeFromReturn());
}

TEST(CommEmergencyStateMachine, ReturnResumeIsCancelledIfLinkDropsAgain)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	machine.update(31'000'000, input, 30'000'000);

	input.link_lost = false;
	input.return_active = true;
	input.resume_distance_allowed = true;
	machine.update(32'000'000, input, 30'000'000);
	input.link_lost = true;
	EXPECT_EQ(machine.update(33'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Committed);
}

TEST(CommEmergencyStateMachine, ContinuesReturnWhenDistanceLimitIsNotSatisfied)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.offboard_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	machine.update(31'000'000, input, 30'000'000);

	input.link_lost = false;
	input.return_active = true;
	input.resume_distance_allowed = false;
	EXPECT_EQ(machine.update(32'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Committed);
}

TEST(CommEmergencyStateMachine, DoesNotRecoverBeforeReturnIsActive)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	machine.update(31'000'000, input, 30'000'000);

	input.link_lost = false;
	input.resume_distance_allowed = true;
	EXPECT_EQ(machine.update(32'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Committed);
}

TEST(CommEmergencyStateMachine, LowBatteryReturnCannotResume)
{
	CommEmergencyStateMachine machine;
	CommEmergencyStateMachine::Input input{};
	input.armed = true;
	input.landed = false;
	input.mission_intended = true;
	input.link_lost = true;
	machine.update(1'000'000, input, 30'000'000);
	input.battery_below_threshold = true;
	input.rtl_feasible = true;
	EXPECT_EQ(machine.update(2'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::Return);

	input.link_lost = false;
	input.return_active = true;
	input.resume_distance_allowed = true;
	EXPECT_EQ(machine.update(3'000'000, input, 30'000'000), CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Committed);
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

	input.link_lost = false;
	input.return_active = true;
	input.resume_distance_allowed = true;
	EXPECT_EQ(machine.update(32'000'000, input, 30'000'000, CommEmergencyStateMachine::Action::Land),
		  CommEmergencyStateMachine::Action::None);
	EXPECT_EQ(machine.state(), CommEmergencyStateMachine::State::Committed);
}

TEST(FlightDistanceTracker, IntegratesActualHorizontalTravel)
{
	FlightDistanceTracker tracker;

	tracker.update(500'000, false, true, true, 0.f, 0.f);
	tracker.update(1'000'000, true, false, true, 10.f, 0.f);

	for (uint64_t sample_time = 2'000'000; sample_time <= 11'000'000; sample_time += 1'000'000) {
		tracker.update(sample_time, true, false, true, 10.f, 0.f);
	}

	EXPECT_TRUE(tracker.valid(11'000'000));
	EXPECT_FLOAT_EQ(tracker.distanceM(), 100.f);
}

TEST(FlightDistanceTracker, StartingInFlightFailsClosed)
{
	FlightDistanceTracker tracker;
	tracker.update(1'000'000, true, false, true, 20.f, 0.f);
	EXPECT_FALSE(tracker.valid(1'000'000));
}

TEST(FlightDistanceTracker, InvalidOrMissingVelocityFailsClosedUntilLanding)
{
	FlightDistanceTracker tracker;
	tracker.update(500'000, false, true, true, 0.f, 0.f);
	tracker.update(1'000'000, true, false, true, 10.f, 0.f);
	tracker.update(2'100'001, true, false, true, 10.f, 0.f);
	EXPECT_FALSE(tracker.valid(2'100'001));

	tracker.update(3'000'000, true, false, true, 10.f, 0.f);
	EXPECT_FALSE(tracker.valid(3'000'000));

	tracker.update(4'000'000, true, true, true, 0.f, 0.f);
	tracker.update(5'000'000, true, false, true, 10.f, 0.f);
	EXPECT_TRUE(tracker.valid(5'000'000));
}
