#include "comm_emergency.hpp"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

CommEmergency::CommEmergency() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool CommEmergency::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void CommEmergency::reset()
{
	_state_machine.reset();
	_pending_action = CommEmergencyStateMachine::Action::None;
	_last_command_time = 0;
	_offboard_prestream_start = 0;
}

void CommEmergency::send_mode_command(CommEmergencyStateMachine::Action action, const vehicle_status_s &status)
{
	uint8_t sub_mode = 0;
	uint8_t main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;

	switch (action) {
	case CommEmergencyStateMachine::Action::Hold:
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case CommEmergencyStateMachine::Action::ResumeMission:
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case CommEmergencyStateMachine::Action::ResumeOffboard:
		main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case CommEmergencyStateMachine::Action::Return:
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case CommEmergencyStateMachine::Action::Land:
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case CommEmergencyStateMachine::Action::None:
		return;
	}

	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f;
	command.param2 = main_mode;
	command.param3 = sub_mode;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_command_time = command.timestamp;
}

bool CommEmergency::resume_action_pending() const
{
	return _pending_action == CommEmergencyStateMachine::Action::ResumeMission
	       || _pending_action == CommEmergencyStateMachine::Action::ResumeOffboard;
}

bool CommEmergency::midcourse_recovery_ready(hrt_abstime now) const
{
	if (!_state_machine.midcourseRecoveryActive()) {
		return true;
	}

	const bool guidance_status_fresh = _dyt_guidance_status.timestamp != 0
					   && _dyt_guidance_status.timestamp <= now
					   && now - _dyt_guidance_status.timestamp < 500_ms;

	return guidance_status_fresh && _dyt_guidance_status.midcourse_target_valid
	       && !_vehicle_status.failsafe;
}

void CommEmergency::publish_status(bool terminal_guidance_inhibited)
{
	comm_emergency_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.active = _state_machine.state() != CommEmergencyStateMachine::State::Idle;
	status.midcourse_recovery_active = _state_machine.midcourseRecoveryActive();
	status.terminal_guidance_inhibited = terminal_guidance_inhibited;
	_status_pub.publish(status);
}

uint8_t CommEmergency::expected_nav_state(CommEmergencyStateMachine::Action action) const
{
	switch (action) {
	case CommEmergencyStateMachine::Action::Hold: return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
	case CommEmergencyStateMachine::Action::ResumeMission: return vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	case CommEmergencyStateMachine::Action::ResumeOffboard: return vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	case CommEmergencyStateMachine::Action::Return: return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	case CommEmergencyStateMachine::Action::Land: return vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
	case CommEmergencyStateMachine::Action::None: return UINT8_MAX;
	}

	return UINT8_MAX;
}

bool CommEmergency::publish_offboard_resume_prestream(hrt_abstime now)
{
	const bool local_position_fresh = _vehicle_local_position.timestamp != 0
					  && _vehicle_local_position.timestamp <= now
					  && now - _vehicle_local_position.timestamp < 200_ms;
	const bool local_position_valid = local_position_fresh && _vehicle_local_position.xy_valid
					  && _vehicle_local_position.z_valid
					  && PX4_ISFINITE(_vehicle_local_position.x)
					  && PX4_ISFINITE(_vehicle_local_position.y)
					  && PX4_ISFINITE(_vehicle_local_position.z);

	if (!local_position_valid) {
		_offboard_prestream_start = 0;
		return false;
	}

	offboard_control_mode_s mode{};
	mode.timestamp = now;
	mode.position = true;
	mode.velocity = false;
	mode.acceleration = false;
	mode.attitude = false;
	mode.body_rate = false;
	mode.thrust_and_torque = false;
	mode.direct_actuator = false;
	_offboard_control_mode_pub.publish(mode);

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.position[0] = _vehicle_local_position.x;
	setpoint.position[1] = _vehicle_local_position.y;
	setpoint.position[2] = _vehicle_local_position.z;

	for (int i = 0; i < 3; ++i) {
		setpoint.velocity[i] = NAN;
		setpoint.acceleration[i] = NAN;
		setpoint.jerk[i] = NAN;
	}

	setpoint.yaw = _vehicle_local_position.heading_good_for_control
		       && PX4_ISFINITE(_vehicle_local_position.heading) ? _vehicle_local_position.heading : NAN;
	setpoint.yawspeed = NAN;
	_trajectory_setpoint_pub.publish(setpoint);

	if (_offboard_prestream_start == 0) {
		_offboard_prestream_start = now;
	}

	return now - _offboard_prestream_start >= 1_s;
}

void CommEmergency::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_land_detected_sub.update(&_land_detected);
	_dyt_guidance_status_sub.update(&_dyt_guidance_status);
	_rtl_time_estimate_sub.update(&_rtl_time_estimate);
	const bool local_position_updated = _vehicle_local_position_sub.update(&_vehicle_local_position);
	const hrt_abstime now = hrt_absolute_time();
	const bool status_fresh = _vehicle_status.timestamp != 0 && _vehicle_status.timestamp <= now
				  && now - _vehicle_status.timestamp < 1_s;
	const bool armed = status_fresh
			   && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool land_status_fresh = _land_detected.timestamp != 0 && _land_detected.timestamp <= now
				       && now - _land_detected.timestamp < CommEmergencyStateMachine::LAND_STATUS_TIMEOUT_US;
	const bool landed_confirmed = land_status_fresh && _land_detected.landed;

	if (status_fresh && (local_position_updated || !armed || landed_confirmed)) {
		const hrt_abstime sample_time = _vehicle_local_position.timestamp_sample != 0 ?
					    _vehicle_local_position.timestamp_sample : _vehicle_local_position.timestamp;
		_flight_distance_tracker.update(sample_time, armed, landed_confirmed,
						_vehicle_local_position.v_xy_valid,
						_vehicle_local_position.vx, _vehicle_local_position.vy);
	}

	if (_param_enable.get() <= 0) {
		reset();
		_gcs_seen = false;
		publish_status();
		return;
	}

	if (!status_fresh) {
		return;
	}

	bool gcs_connected = false;

	for (auto &telemetry_sub : _telemetry_status_subs) {
		telemetry_status_s telemetry{};

		if (telemetry_sub.copy(&telemetry) && telemetry.timestamp != 0
		    && now - telemetry.timestamp < 3_s && telemetry.heartbeat_type_gcs) {
			gcs_connected = true;
		}
	}

	if (gcs_connected) {
		_gcs_seen = true;
	}

	bool battery_remaining_valid = false;
	bool battery_time_remaining_valid = true;
	_battery_remaining = 1.f;
	_battery_time_remaining_s = INFINITY;

	for (auto &battery_sub : _battery_status_subs) {
		battery_status_s battery{};

		if (!battery_sub.copy(&battery) || !battery.connected || battery.timestamp == 0
		    || now - battery.timestamp >= 1_s) {
			continue;
		}

		if (PX4_ISFINITE(battery.remaining) && battery.remaining >= 0.f && battery.remaining <= 1.f) {
			battery_remaining_valid = true;
			_battery_remaining = math::min(_battery_remaining, battery.remaining);

			if (PX4_ISFINITE(battery.time_remaining_s) && battery.time_remaining_s >= 0.f) {
				_battery_time_remaining_s = math::min(_battery_time_remaining_s, battery.time_remaining_s);

			} else {
				battery_time_remaining_valid = false;
			}
		}
	}

	if (!battery_remaining_valid) {
		_battery_remaining = NAN;
		battery_time_remaining_valid = false;
	}

	if (!battery_time_remaining_valid) {
		_battery_time_remaining_s = NAN;
	}

	const bool rtl_estimate_valid = _rtl_time_estimate.timestamp != 0
					&& now - _rtl_time_estimate.timestamp < 3_s
					&& _rtl_time_estimate.valid
					&& PX4_ISFINITE(_rtl_time_estimate.safe_time_estimate)
					&& _rtl_time_estimate.safe_time_estimate >= 0.f;
	_rtl_feasible = battery_time_remaining_valid && rtl_estimate_valid
			&& _battery_time_remaining_s > _rtl_time_estimate.safe_time_estimate;

	CommEmergencyStateMachine::Input input{};
	input.armed = armed;
	input.landed = CommEmergencyStateMachine::landedOrStatusUnavailable(now, _land_detected.timestamp,
			_land_detected.landed);
	input.mission_intended =
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	input.offboard_intended =
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	input.link_lost = (_gcs_seen && !gcs_connected) || _vehicle_status.gcs_connection_lost;
	const float battery_threshold = math::constrain(_param_battery_threshold.get(), 0.05f, 0.95f);
	input.battery_below_threshold = battery_remaining_valid && _battery_remaining < battery_threshold;
	input.rtl_feasible = _rtl_feasible;
	input.return_active = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	const bool guidance_status_fresh = _dyt_guidance_status.timestamp != 0
					   && _dyt_guidance_status.timestamp <= now
					   && now - _dyt_guidance_status.timestamp < 500_ms;
	const bool terminal_guidance_active = guidance_status_fresh
					      && _dyt_guidance_status.guidance_phase
					      == dyt_guidance_status_s::PHASE_TERMINAL;

	if (terminal_guidance_active) {
		reset();
		publish_status(true);
		return;
	}

	input.midcourse_active = guidance_status_fresh
				 && _dyt_guidance_status.guidance_phase == dyt_guidance_status_s::PHASE_MIDCOURSE
				 && _dyt_guidance_status.midcourse_active;
	input.midcourse_requested = !_vehicle_status.failsafe && guidance_status_fresh
				    && _dyt_guidance_status.gcs_phase_request == dyt_guidance_status_s::PHASE_MIDCOURSE
				    && _dyt_guidance_status.command_phase == dyt_guidance_status_s::PHASE_MIDCOURSE
				    && _dyt_guidance_status.command_result == dyt_guidance_status_s::COMMAND_RESULT_PENDING;
	// Resume the original Mission/Offboard only below 2 km of accumulated horizontal flight distance.
	input.resume_distance_allowed = !_vehicle_status.failsafe && _flight_distance_tracker.valid(now)
					&& _flight_distance_tracker.distanceM() < RESUME_DISTANCE_LIMIT_M;

	const float wait_s = math::constrain(_param_wait_s.get(), 5.f, 600.f);
	const uint64_t wait_us = static_cast<uint64_t>(wait_s * 1_s);
	const CommEmergencyStateMachine::Action timeout_action = _param_timeout_action.get() == 1 ?
		CommEmergencyStateMachine::Action::Land : CommEmergencyStateMachine::Action::Return;

	if (_state_machine.state() == CommEmergencyStateMachine::State::Resuming && resume_action_pending()
	    && _vehicle_status.nav_state == expected_nav_state(_pending_action)) {
		_state_machine.resumeCompleted();
		_pending_action = CommEmergencyStateMachine::Action::None;
		_last_command_time = 0;
		_offboard_prestream_start = 0;
	}

	const CommEmergencyStateMachine::State state_before_update = _state_machine.state();
	const CommEmergencyStateMachine::Action action = _state_machine.update(now, input, wait_us, timeout_action);
	publish_status();

	if (action != CommEmergencyStateMachine::Action::None) {
		_pending_action = action;
		_last_command_time = 0;
		_offboard_prestream_start = 0;

		if (_state_machine.resumeFromReturn()) {
			PX4_WARN("communication restored: resume action=%u distance=%.1f m",
				 static_cast<unsigned>(action), (double)_flight_distance_tracker.distanceM());

		} else {
			PX4_WARN("communication emergency action: %u", static_cast<unsigned>(action));
		}

	} else if (state_before_update == CommEmergencyStateMachine::State::Resuming
		   && _state_machine.state() != CommEmergencyStateMachine::State::Resuming
		   && resume_action_pending()) {
		_pending_action = CommEmergencyStateMachine::Action::None;
		_last_command_time = 0;
		_offboard_prestream_start = 0;

	} else if (!input.link_lost && _state_machine.state() == CommEmergencyStateMachine::State::Idle
		   && _pending_action == CommEmergencyStateMachine::Action::Hold) {
		// A non-mission flight has no autonomous mode to resume. Stop retrying an
		// unacknowledged Hold command once the link is healthy again.
		_pending_action = CommEmergencyStateMachine::Action::None;
	}

	if (_pending_action != CommEmergencyStateMachine::Action::None) {
		if (_vehicle_status.nav_state == expected_nav_state(_pending_action)) {
			if (resume_action_pending()) {
				_state_machine.resumeCompleted();
			}

			_pending_action = CommEmergencyStateMachine::Action::None;
			_last_command_time = 0;
			_offboard_prestream_start = 0;

		} else if (_pending_action == CommEmergencyStateMachine::Action::ResumeOffboard) {
			if (_state_machine.state() != CommEmergencyStateMachine::State::Resuming) {
				_pending_action = CommEmergencyStateMachine::Action::None;
				_offboard_prestream_start = 0;

			} else if (!midcourse_recovery_ready(now)) {
				// Require continuously fresh target data throughout the complete Offboard pre-stream interval.
				_offboard_prestream_start = 0;

			} else if (publish_offboard_resume_prestream(now)
				   && (_last_command_time == 0 || now - _last_command_time >= 1_s)) {
				send_mode_command(_pending_action, _vehicle_status);
			}

		} else if (_last_command_time == 0 || now - _last_command_time >= 1_s) {
			send_mode_command(_pending_action, _vehicle_status);
		}
	}
}

int CommEmergency::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("state=%u pending=%u loss_age=%.1f s battery=%.1f%% rtl=%s distance=%.1f m valid=%d",
		 static_cast<unsigned>(_state_machine.state()),
		 static_cast<unsigned>(_pending_action),
		 _state_machine.lossStarted() == 0 ? -1.0 : (double)(now - _state_machine.lossStarted()) * 1e-6,
		 (double)(_battery_remaining * 100.f), _rtl_feasible ? "feasible" : "not feasible",
		 (double)_flight_distance_tracker.distanceM(), _flight_distance_tracker.valid(now));
	return 0;
}

int CommEmergency::task_spawn(int argc, char *argv[])
{
	CommEmergency *instance = new CommEmergency();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int CommEmergency::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CommEmergency::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Communication-loss hold, energy-aware return or land, and distance-gated mission/offboard recovery.");
	PRINT_MODULE_USAGE_NAME("comm_emergency", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int comm_emergency_main(int argc, char *argv[])
{
	return CommEmergency::main(argc, argv);
}
