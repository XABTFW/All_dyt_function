#pragma once

#include "CommEmergencyStateMachine.hpp"
#include "FlightDistanceTracker.hpp"

#include <math.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/dyt_guidance_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class CommEmergency : public ModuleBase<CommEmergency>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CommEmergency();
	~CommEmergency() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void send_mode_command(CommEmergencyStateMachine::Action action, const vehicle_status_s &status);
	bool publish_offboard_resume_prestream(hrt_abstime now);
	uint8_t expected_nav_state(CommEmergencyStateMachine::Action action) const;
	void reset();
	bool resume_action_pending() const;

	static constexpr float RESUME_DISTANCE_LIMIT_M = 2000.f;

	CommEmergencyStateMachine _state_machine{};
	FlightDistanceTracker _flight_distance_tracker{};
	CommEmergencyStateMachine::Action _pending_action{CommEmergencyStateMachine::Action::None};
	hrt_abstime _last_command_time{0};
	hrt_abstime _offboard_prestream_start{0};
	float _battery_remaining{NAN};
	float _battery_time_remaining_s{NAN};
	bool _rtl_feasible{false};
	bool _gcs_seen{false};

	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land_detected{};
	dyt_guidance_status_s _dyt_guidance_status{};
	rtl_time_estimate_s _rtl_time_estimate{};
	vehicle_local_position_s _vehicle_local_position{};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _dyt_guidance_status_sub{ORB_ID(dyt_guidance_status)};
	uORB::Subscription _rtl_time_estimate_sub{ORB_ID(rtl_time_estimate)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{
		ORB_ID::battery_status};
	uORB::SubscriptionMultiArray<telemetry_status_s> _telemetry_status_subs{ORB_ID::telemetry_status};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CEM_EN>) _param_enable,
		(ParamFloat<px4::params::CEM_WAIT>) _param_wait_s,
		(ParamInt<px4::params::CEM_TO_ACT>) _param_timeout_action,
		(ParamFloat<px4::params::CEM_BAT_THR>) _param_battery_threshold
	)
};
