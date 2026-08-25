#pragma once

#include "GnssEmergencyStateMachine.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class GnssEmergency : public ModuleBase<GnssEmergency>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GnssEmergency();
	~GnssEmergency() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void send_mode_command(GnssEmergencyStateMachine::Action action, const vehicle_status_s &status);
	void reset();

	GnssEmergencyStateMachine _state_machine{};
	GnssEmergencyStateMachine::Action _pending_action{GnssEmergencyStateMachine::Action::None};
	hrt_abstime _last_command_time{0};
	bool _gps_seen_healthy{false};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	sensor_gps_s _sensor_gps{};
	failsafe_flags_s _failsafe_flags{};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::GEM_EN>) _param_enable,
		(ParamFloat<px4::params::GEM_REC_T>) _param_recovery_time_s
	)
};
