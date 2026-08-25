#include "gnss_emergency.hpp"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
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
}

void GnssEmergency::send_mode_command(GnssEmergencyStateMachine::Action action, const vehicle_status_s &status)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();

	switch (action) {
	case GnssEmergencyStateMachine::Action::Descend:
		// DESCEND keeps the vehicle armed and does not require horizontal position.
		// It also avoids treating the recoverable GNSS emergency as a user-requested AUTO_LAND.
		command.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
		command.param1 = vehicle_status_s::NAVIGATION_STATE_DESCEND;
		break;

	case GnssEmergencyStateMachine::Action::ResumeMission:
		command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		command.param1 = 1.f;
		command.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
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

	if (_param_enable.get() <= 0) {
		reset();
		_gps_seen_healthy = false;
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
		_gps_seen_healthy = false;
		return;
	}

	const bool gps_fresh = _sensor_gps.timestamp != 0 && now - _sensor_gps.timestamp < 1_s;
	const bool explicit_interference = gps_fresh &&
		(_sensor_gps.jamming_state == sensor_gps_s::JAMMING_STATE_DETECTED ||
		 _sensor_gps.spoofing_state >= sensor_gps_s::SPOOFING_STATE_MITIGATED);
	const bool gps_usable = gps_fresh && _sensor_gps.fix_type >= sensor_gps_s::FIX_TYPE_3D && !explicit_interference;

	if (gps_usable) {
		_gps_seen_healthy = true;
	}

	const bool emergency_mode_intended =
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND ||
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_DESCEND;

	if (_state_machine.state() == GnssEmergencyStateMachine::State::Landing && !emergency_mode_intended) {
		PX4_WARN("GNSS emergency released by mode change");
		reset();
		return;
	}

	const bool flags_fresh = _failsafe_flags.timestamp != 0 && now - _failsafe_flags.timestamp < 1_s;
	const bool navigation_recovered = gps_usable && flags_fresh &&
		!_failsafe_flags.local_position_invalid && !_failsafe_flags.global_position_invalid;
	const bool interference = explicit_interference || (_gps_seen_healthy && !gps_usable);
	const bool landed = GnssEmergencyStateMachine::landedOrStatusUnavailable(now, _land_detected.timestamp,
			    _land_detected.landed);

	GnssEmergencyStateMachine::Input input{};
	input.armed = armed;
	input.landed = landed;
	input.mission_active =
		_vehicle_status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	input.interference = interference;
	input.navigation_recovered = navigation_recovered;

	const float recovery_time_s = math::constrain(_param_recovery_time_s.get(), 1.f, 30.f);
	const uint64_t recovery_time_us = static_cast<uint64_t>(recovery_time_s * 1_s);
	const GnssEmergencyStateMachine::Action action = _state_machine.update(now, input, recovery_time_us);

	if (action != GnssEmergencyStateMachine::Action::None) {
		_pending_action = action;
		PX4_WARN("GNSS emergency action: %u", static_cast<unsigned>(action));
	}

	if (_pending_action != GnssEmergencyStateMachine::Action::None) {
		const bool landing_active = _pending_action == GnssEmergencyStateMachine::Action::Descend
					    && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND);
		const bool mission_active = _pending_action == GnssEmergencyStateMachine::Action::ResumeMission
					    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

		if (landing_active || mission_active) {
			_pending_action = GnssEmergencyStateMachine::Action::None;

		} else if (_last_command_time == 0 || now - _last_command_time >= 1_s) {
			send_mode_command(_pending_action, _vehicle_status);
		}
	}
}

int GnssEmergency::print_status()
{
	PX4_INFO("state=%u pending=%u gps_seen=%d recovery_age=%.1f s", static_cast<unsigned>(_state_machine.state()),
		 static_cast<unsigned>(_pending_action), _gps_seen_healthy,
		 _state_machine.recoveryStarted() == 0 ? -1.0 :
		 (double)(hrt_absolute_time() - _state_machine.recoveryStarted()) * 1e-6);
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

	PRINT_MODULE_DESCRIPTION("GNSS jamming, spoofing and loss landing/descend with mission recovery before touchdown.");
	PRINT_MODULE_USAGE_NAME("gnss_emergency", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gnss_emergency_main(int argc, char *argv[])
{
	return GnssEmergency::main(argc, argv);
}
