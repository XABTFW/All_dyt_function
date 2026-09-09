#include "gnss_emergency.hpp"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

GnssEmergency::GnssEmergency() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool GnssEmergency::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void GnssEmergency::reset()
{
	_state_machine.reset();
	_pending_action = GnssEmergencyStateMachine::Action::None;
	_last_command_time = 0;
	_last_rc_mode_request = 0;
}

void GnssEmergency::send_mode_command(GnssEmergencyStateMachine::Action action, const vehicle_status_s &status)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();

	switch (action) {
	case GnssEmergencyStateMachine::Action::Land:
		command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		command.param1 = 1.f;
		command.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case GnssEmergencyStateMachine::Action::None:
		return;
	}

	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_command_time = command.timestamp;
}

void GnssEmergency::Run()
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
	_sensor_gps_sub.update(&_sensor_gps);
	_failsafe_flags_sub.update(&_failsafe_flags);

	if (_action_request_sub.update(&_action_request)
	    && _action_request.action == action_request_s::ACTION_SWITCH_MODE
	    && (_action_request.source == action_request_s::SOURCE_RC_SWITCH
		|| _action_request.source == action_request_s::SOURCE_RC_MODE_SLOT)) {
		_last_rc_mode_request = _action_request.timestamp;
	}

	if (_param_enable.get() <= 0) {
		reset();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool status_fresh = _vehicle_status.timestamp != 0 && now - _vehicle_status.timestamp < 1_s;

	if (!status_fresh) {
		return;
	}

	const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (!armed) {
		reset();
		return;
	}

	const bool gps_available = _sensor_gps.timestamp != 0 && _sensor_gps.timestamp <= now
				   && now - _sensor_gps.timestamp < 3_s && _sensor_gps.satellites_used > 0;

	const bool flags_fresh = _failsafe_flags.timestamp != 0 && now - _failsafe_flags.timestamp < 1_s;
	const bool manual_control_available = flags_fresh && !_failsafe_flags.manual_control_signal_lost;
	const bool rc_mode_request_fresh = _last_rc_mode_request != 0 && _last_rc_mode_request <= now
					   && now - _last_rc_mode_request < 1_s;
	const bool manual_takeover = manual_control_available && rc_mode_request_fresh;
	const bool landed = GnssEmergencyStateMachine::landedOrStatusUnavailable(now, _land_detected.timestamp,
			    _land_detected.landed);

	GnssEmergencyStateMachine::Input input{};
	input.armed = armed;
	input.landed = landed;
	input.gnss_failure = !gps_available;
	input.manual_control_available = manual_control_available;
	input.manual_takeover = manual_takeover;

	const GnssEmergencyStateMachine::State state_before_update = _state_machine.state();
	const GnssEmergencyStateMachine::Action action = _state_machine.update(input);

	if (state_before_update == GnssEmergencyStateMachine::State::Landing
	    && _state_machine.state() == GnssEmergencyStateMachine::State::Released) {
		PX4_WARN("GNSS emergency released by RC mode change");
		_pending_action = GnssEmergencyStateMachine::Action::None;
		_last_command_time = 0;
	}

	if (action != GnssEmergencyStateMachine::Action::None) {
		_pending_action = action;
		PX4_WARN("GNSS emergency action: %u", static_cast<unsigned>(action));
	}

	const bool landing_active = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
				    || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND;

	if (_state_machine.state() == GnssEmergencyStateMachine::State::Landing
	    && _pending_action == GnssEmergencyStateMachine::Action::None && !landing_active) {
		_pending_action = GnssEmergencyStateMachine::Action::Land;
		_last_command_time = 0;
	}

	if (_pending_action != GnssEmergencyStateMachine::Action::None) {
		if (_pending_action == GnssEmergencyStateMachine::Action::Land && landing_active) {
			_pending_action = GnssEmergencyStateMachine::Action::None;

		} else if (_last_command_time == 0 || now - _last_command_time >= 1_s) {
			send_mode_command(_pending_action, _vehicle_status);
		}
	}
}

int GnssEmergency::print_status()
{
	PX4_INFO("state=%u pending=%u", static_cast<unsigned>(_state_machine.state()),
		 static_cast<unsigned>(_pending_action));
	return 0;
}

int GnssEmergency::task_spawn(int argc, char *argv[])
{
	GnssEmergency *instance = new GnssEmergency();

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

int GnssEmergency::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GnssEmergency::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("GNSS three-second data loss and zero-satellite landing with RC takeover.");
	PRINT_MODULE_USAGE_NAME("gnss_emergency", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gnss_emergency_main(int argc, char *argv[])
{
	return GnssEmergency::main(argc, argv);
}
