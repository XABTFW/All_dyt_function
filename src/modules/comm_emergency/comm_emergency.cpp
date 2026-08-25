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
}

void CommEmergency::send_mode_command(CommEmergencyStateMachine::Action action, const vehicle_status_s &status)
{
	uint8_t sub_mode = 0;

	switch (action) {
	case CommEmergencyStateMachine::Action::Hold:
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case CommEmergencyStateMachine::Action::ResumeMission:
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
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
	command.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = sub_mode;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_command_time = command.timestamp;
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
	_rtl_time_estimate_sub.update(&_rtl_time_estimate);

	if (_param_enable.get() <= 0) {
		reset();
		_gcs_seen = false;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool status_fresh = _vehicle_status.timestamp != 0 && now - _vehicle_status.timestamp < 1_s;

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
	input.armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	input.landed = CommEmergencyStateMachine::landedOrStatusUnavailable(now, _land_detected.timestamp,
			_land_detected.landed);
	input.mission_intended =
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	input.link_lost = (_gcs_seen && !gcs_connected) || _vehicle_status.gcs_connection_lost;
	const float battery_threshold = math::constrain(_param_battery_threshold.get(), 0.05f, 0.95f);
	input.battery_below_threshold = battery_remaining_valid && _battery_remaining < battery_threshold;
	input.rtl_feasible = _rtl_feasible;

	const float wait_s = math::constrain(_param_wait_s.get(), 5.f, 600.f);
	const uint64_t wait_us = static_cast<uint64_t>(wait_s * 1_s);
	const CommEmergencyStateMachine::Action timeout_action = _param_timeout_action.get() == 1 ?
		CommEmergencyStateMachine::Action::Land : CommEmergencyStateMachine::Action::Return;
	const CommEmergencyStateMachine::Action action = _state_machine.update(now, input, wait_us, timeout_action);

	if (action != CommEmergencyStateMachine::Action::None) {
		_pending_action = action;
		PX4_WARN("communication emergency action: %u", static_cast<unsigned>(action));

	} else if (!input.link_lost && _state_machine.state() == CommEmergencyStateMachine::State::Idle) {
		// A non-mission flight has no autonomous mode to resume. Stop retrying an
		// unacknowledged Hold command once the link is healthy again.
		_pending_action = CommEmergencyStateMachine::Action::None;
	}

	uint8_t expected_nav_state = UINT8_MAX;

	switch (_pending_action) {
	case CommEmergencyStateMachine::Action::Hold: expected_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER; break;
	case CommEmergencyStateMachine::Action::ResumeMission: expected_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION; break;
	case CommEmergencyStateMachine::Action::Return: expected_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL; break;
	case CommEmergencyStateMachine::Action::Land: expected_nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND; break;
	case CommEmergencyStateMachine::Action::None: break;
	}

	if (_pending_action != CommEmergencyStateMachine::Action::None) {
		if (_vehicle_status.nav_state == expected_nav_state) {
			_pending_action = CommEmergencyStateMachine::Action::None;

		} else if (_last_command_time == 0 || now - _last_command_time >= 1_s) {
			send_mode_command(_pending_action, _vehicle_status);
		}
	}
}

int CommEmergency::print_status()
{
	PX4_INFO("state=%u pending=%u loss_age=%.1f s battery=%.1f%% rtl=%s", static_cast<unsigned>(_state_machine.state()),
		 static_cast<unsigned>(_pending_action),
		 _state_machine.lossStarted() == 0 ? -1.0 : (double)(hrt_absolute_time() - _state_machine.lossStarted()) * 1e-6,
		 (double)(_battery_remaining * 100.f), _rtl_feasible ? "feasible" : "not feasible");
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

	PRINT_MODULE_DESCRIPTION("Communication-loss hold, energy-aware return or land, and mission recovery policy.");
	PRINT_MODULE_USAGE_NAME("comm_emergency", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int comm_emergency_main(int argc, char *argv[])
{
	return CommEmergency::main(argc, argv);
}
