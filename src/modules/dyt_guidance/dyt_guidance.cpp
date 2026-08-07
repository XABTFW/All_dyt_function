/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <math.h>
#include <string.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/dyt_command.h>
#include <uORB/topics/dyt_guidance_status.h>
#include <uORB/topics/dyt_target.h>
#include <uORB/topics/follower_info.h>
#include <uORB/topics/gripper.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class DytGuidance : public ModuleBase<DytGuidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DytGuidance();
	~DytGuidance() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	void show_status();

private:
	static constexpr int OBS_BUFFER_LEN{6};
	static constexpr float TARGET_MIN_BBOX_PX{4.f};
	static constexpr float TARGET_MAX_LOS_RAD{0.45f};
	static constexpr float TARGET_MAX_HINT_LOS_RAD{0.35f};
	static constexpr float TARGET_MAX_GIMBAL_YAW_RAD{1.92f};
	static constexpr float FRAME_YAW_MIN_DEG{-110.f};
	static constexpr float FRAME_YAW_MAX_DEG{110.f};
	static constexpr float FRAME_PITCH_MIN_DEG{-100.f};
	static constexpr float FRAME_PITCH_MAX_DEG{70.f};
	static constexpr float MIDCOURSE_RESEND_ANGLE_DELTA_DEG{0.5f};
	static constexpr int MIDCOURSE_BURST_COUNT{3};
	static constexpr hrt_abstime MIDCOURSE_BURST_INTERVAL{40_ms};
	static constexpr hrt_abstime MIDCOURSE_HOLD_INTERVAL{40_ms};
	static constexpr hrt_abstime MIDCOURSE_OWNSHIP_INTERVAL{40_ms};
	static constexpr hrt_abstime MIDCOURSE_GEO_TARGET_INTERVAL{100_ms};
	static constexpr int SEARCH_CENTER_PASSES{2};
	static constexpr hrt_abstime MANUAL_TAKEOVER_GRACE{500_ms};
	static constexpr hrt_abstime TRACK_HANDOFF_SETPOINT_MAX_AGE{300_ms};

	struct ScanArea {
		float yaw_min_deg{0.f};
		float yaw_max_deg{0.f};
		float pitch_top_deg{0.f};
		float pitch_bottom_deg{0.f};
	};

	struct LosObservation {
		hrt_abstime sample_time{0};
		Vector3f los_body{};
		float frame_dt_s{0.f};
	};

	struct TrackProfile {
		float nav_gain{0.f};
		float v_cmd{0.f};
		float k_a{0.f};
		float k_v{0.f};
		uint8_t submode{dyt_guidance_status_s::SUBMODE_FOLLOW};
	};

	enum class TaskState : uint8_t {
		Idle = dyt_guidance_status_s::STATE_IDLE,
		SearchWaitLock = dyt_guidance_status_s::STATE_SEARCH_WAIT_LOCK,
		TrackFollow = dyt_guidance_status_s::STATE_TRACK_FOLLOW,
		TrackIntercept = dyt_guidance_status_s::STATE_TRACK_INTERCEPT,
		LostHold = dyt_guidance_status_s::STATE_LOST_HOLD,
		Abort = dyt_guidance_status_s::STATE_ABORT
	};

	enum class ScanRegion : uint8_t {
		Center,
		Global
	};

	void Run() override;

	void update_subscriptions();
	void update_vehicle_id();
	void update_params_if_needed();

	float aux_value(int index) const;
	bool aux_switch_active(int index) const;
	bool button_active(int button) const;
	bool payload_switch_active() const;
	bool activation_requested() const;
	bool midcourse_pointing_requested() const;
	bool manual_fire_requested() const;
	bool preconditions_ok() const;
	bool manual_takeover_detected() const;

	void handle_new_target(const dyt_target_s &target);
	bool build_los_body(const dyt_target_s &target, Vector3f &los_body) const;
	void push_observation(const Vector3f &los_body, hrt_abstime sample_time, float frame_dt_s);
	void clear_observations();
	bool update_los_estimate(hrt_abstime now);

	void capture_hold_setpoint();
	void publish_hold_setpoint();
	void publish_track_setpoint(const TrackProfile &profile);
	void publish_offboard_mode(bool position_mode);
	void request_offboard_mode();
	void publish_status();
	void capture_track_handoff_velocity();
	float track_handoff_blend(hrt_abstime now) const;

	void enter_state(TaskState new_state, uint8_t lost_reason = dyt_guidance_status_s::LOST_REASON_NONE);
	bool activate_guidance(hrt_abstime now);
	void activate_guidance_and_request_lock(hrt_abstime now);
	void deactivate_guidance(uint8_t lost_reason);
	void abort_guidance(uint8_t lost_reason);
	void enter_lost_hold(uint8_t lost_reason);
	bool update_lost_reacquire(hrt_abstime now, hrt_abstime lost_enter_time = 0);
	void update_payload_only_reacquire(hrt_abstime now);

	bool target_locked() const;
	bool target_geometry_valid(const dyt_target_s &target) const;
	bool target_geometry_valid() const;
	bool target_hint_detected() const;
	bool target_fresh() const;
	bool target_lock_candidate() const;
	bool target_usable() const;
	bool intercept_allowed() const;
	void update_net_release_trigger(hrt_abstime now);
	bool update_laser_distance(hrt_abstime now);
	void send_net_release_command(hrt_abstime now);
	void clear_net_release_trigger();
	void update_gripper_release_trigger(hrt_abstime now);
	void start_net_hold_after_release();
	void clear_net_hold();
	void publish_net_brake_setpoint();
	void clear_net_decel();
	void start_net_decel_if_ready(hrt_abstime now, const Vector3f &vehicle_velocity);
	void apply_net_decel_velocity_scale(Vector2f &vel_xy) const;
	Vector3f net_decel_accel_ned(hrt_abstime now, const Vector3f &vehicle_velocity);
	bool midcourse_handoff_active() const;
	bool vehicle_control_active() const;
	TrackProfile follow_profile() const;
	TrackProfile intercept_profile() const;

	void send_dyt_command(uint8_t command, int16_t param_x = 0, int16_t param_y = 0, uint8_t param3 = 0, int8_t zoom_rate = 0);
	void send_dyt_ownship_state(hrt_abstime now);
	void send_dyt_geo_track_target(hrt_abstime now, bool force = false);
	void send_dyt_geo_track_exit();
	void send_home_angle_command();
	int16_t angle_deg_to_cdeg(float angle_deg) const;
	bool global_position_valid() const;
	bool local_position_global_valid() const;
	bool midcourse_target_geo_valid() const;
	float midcourse_target_command_alt() const;
	float midcourse_target_relative_alt() const;
	float own_midcourse_relative_alt() const;
	bool midcourse_target_position_local(Vector3f &target_position) const;
	bool compute_midcourse_gimbal_angle(float &yaw_deg, float &pitch_deg) const;
	bool compute_midcourse_inertial_angles(const Vector3f &los_ned, float &yaw_deg, float &pitch_deg) const;
	Vector3f rotate_mount_los_to_body(const Vector3f &los_mount) const;
	float finite_param_deg(float value) const;
	bool update_midcourse_pointing(hrt_abstime now, bool force = false);
	bool update_midcourse_geo_tracking(hrt_abstime now, bool force = false);
	bool update_midcourse_gimbal_pointing(hrt_abstime now, bool force = false);
	void retry_autolock(hrt_abstime now);
	bool update_hint_autolock(hrt_abstime now);
	void update_auto_activation(hrt_abstime now);
	bool handle_lock_candidate_or_timeout(hrt_abstime now);
	void reset_search_scan(hrt_abstime now);
	void update_search_scan(hrt_abstime now);
	void advance_search_scan_area(float pitch_step_deg);
	void send_angle_command(float yaw_deg, float pitch_deg, uint8_t command);
	int scan_row_count(const ScanArea &area, float pitch_step_deg) const;
	ScanArea active_scan_area() const;

	TaskState _state{TaskState::Idle};
	uint8_t _lost_reason{dyt_guidance_status_s::LOST_REASON_NONE};
	uint8_t _requested_submode{dyt_guidance_status_s::SUBMODE_FOLLOW};
	uint8_t _last_command{dyt_command_s::CMD_NONE};

	bool _prev_activation_request{false};
	bool _manual_activation{false};
	bool _payload_lock_seen{false};
	bool _payload_lost_hold{false};
	bool _midcourse_handoff_latched{false};
	int _lock_streak{0};
	int _relock_streak{0};
	int _lost_streak{0};
	int _auto_lock_streak{0};
	ScanRegion _scan_region{ScanRegion::Center};
	int _scan_center_passes{0};
	int _scan_row{0};
	float _scan_segment_target_deg{NAN};

	hrt_abstime _state_enter_time{0};
	hrt_abstime _payload_lost_enter_time{0};
	hrt_abstime _last_offboard_request{0};
	hrt_abstime _last_home_command_time{0};
	hrt_abstime _last_retrigger_time{0};
	hrt_abstime _last_hint_lock_time{0};
	hrt_abstime _search_pause_until{0};
	hrt_abstime _next_scan_time{0};
	hrt_abstime _last_command_time{0};
	hrt_abstime _candidate_lock_start_time{0};
	hrt_abstime _candidate_ignore_until{0};
	hrt_abstime _candidate_ignored_sample_time{0};
	hrt_abstime _auto_lock_last_sample_time{0};
	hrt_abstime _last_laser_sample_time{0};
	hrt_abstime _net_hold_start_time{0};
	hrt_abstime _net_decel_until{0};
	bool _candidate_lock_active{false};
	bool _net_release_sent{false};
	bool _prev_manual_fire_request{false};
	bool _net_hold_active{false};
	bool _net_brake_active{false};
	bool _net_decel_pending{false};
	bool _net_decel_low_speed_active{false};
	uint32_t _command_pub_count{0};
	float _scan_yaw_deg{0.f};
	float _scan_pitch_deg{0.f};
	float _laser_distance_m{NAN};
	float _net_decel_initial_speed{0.f};
	float _net_decel_target_speed{0.f};
	hrt_abstime _next_midcourse_point_time{0};
	hrt_abstime _last_midcourse_point_time{0};
	hrt_abstime _last_midcourse_ownship_time{0};
	hrt_abstime _last_midcourse_geo_target_time{0};
	hrt_abstime _track_handoff_time{0};
	float _midcourse_yaw_deg{NAN};
	float _midcourse_pitch_deg{NAN};
	int _midcourse_burst_remaining{0};
	bool _midcourse_geotrack_active{false};
	bool _track_handoff_velocity_valid{false};

	LosObservation _observations[OBS_BUFFER_LEN]{};
	int _observation_count{0};

	dyt_target_s _last_target{};
	bool _have_target{false};
	follower_info_s _midcourse_target_info{};
	hrt_abstime _last_midcourse_target_time{0};
	float _midcourse_target_alt_ref_m{NAN};
	uint32_t _midcourse_target_alt_ref_mavid{0};
	uint32_t _vehicle_id{0};
	bool _vehicle_id_initialized{false};

	vehicle_attitude_s _vehicle_attitude{};
	vehicle_global_position_s _vehicle_global_position{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_local_position_setpoint_s _vehicle_local_position_setpoint{};
	vehicle_status_s _vehicle_status{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	airspeed_validated_s _airspeed_validated{};
	manual_control_setpoint_s _manual_control{};
	manual_control_switches_s _manual_switches{};

	Vector3f _hold_position{};
	float _hold_yaw{0.f};
	Vector3f _track_handoff_velocity{};

	Vector3f _los_ned{};
	Vector3f _los_body_latest{};
	Vector3f _los_filtered{};
	Vector3f _prev_los_filtered{};
	Vector3f _omega_los{};
	Vector3f _velocity_sp{};
	Vector3f _acceleration_sp{};
	float _yaw_sp{NAN};
	float _yaw_rate_sp{NAN};
	hrt_abstime _prev_los_update{0};
	hrt_abstime _last_track_setpoint_time{0};
	bool _los_filter_initialized{false};

	uORB::Subscription _dyt_target_sub{ORB_ID(dyt_target)};
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	uORB::Subscription _follower_info_sub{ORB_ID(follower_info)};
	uORB::Subscription _gripper_sub{ORB_ID(gripper)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _manual_switches_sub{ORB_ID(manual_control_switches)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<dyt_command_s> _dyt_command_pub{ORB_ID(dyt_command)};
	uORB::Publication<dyt_guidance_status_s> _dyt_guidance_status_pub{ORB_ID(dyt_guidance_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DYTG_ACT_AUX>) _param_act_aux,
		(ParamInt<px4::params::DYTG_ACT_BTN>) _param_act_btn,
		(ParamInt<px4::params::DYTG_INT_AUX>) _param_int_aux,
		(ParamInt<px4::params::DYTG_COOP_EN>) _param_coop_enable,
		(ParamInt<px4::params::DYTG_GEO_EN>) _param_midcourse_geo_enable,
		(ParamInt<px4::params::CRDZ_ACT_AUX>) _param_midcourse_act_aux,
		(ParamInt<px4::params::CRDZ_ACT_BTN>) _param_midcourse_act_btn,
		(ParamInt<px4::params::DYTG_TGT_ID>) _param_midcourse_target_id,
		(ParamFloat<px4::params::DYTG_TGT_TO>) _param_midcourse_target_timeout,
		(ParamInt<px4::params::DYTG_ALT_MODE>) _param_midcourse_alt_mode,
		(ParamFloat<px4::params::DYTG_TGT_ALTOFF>) _param_midcourse_target_alt_offset,
		(ParamFloat<px4::params::DYTG_MNT_PRED>) _param_midcourse_gimbal_prediction,
		(ParamInt<px4::params::DYTG_MNT_MODE>) _param_midcourse_gimbal_mode,
		(ParamFloat<px4::params::DYTG_STK_TK>) _param_stick_takeover,
		(ParamInt<px4::params::DYTG_AUTO_EN>) _param_auto_enable,
		(ParamInt<px4::params::DYTG_AUTO_N>) _param_auto_frames,
		(ParamInt<px4::params::DYTG_LOCK_N>) _param_lock_frames,
		(ParamInt<px4::params::DYTG_RELOCKN>) _param_relock_frames,
		(ParamInt<px4::params::DYTG_WAITMS>) _param_wait_ms,
		(ParamInt<px4::params::DYTG_LOSTMS>) _param_lost_ms,
		(ParamFloat<px4::params::DYTG_SC_YSPD>) _param_scan_yaw_speed,
		(ParamFloat<px4::params::DYTG_SC_PSPD>) _param_scan_pitch_speed,
		(ParamFloat<px4::params::DYTG_SC_STEP>) _param_scan_pitch_step,
		(ParamFloat<px4::params::DYTG_SC_PAUSE>) _param_scan_edge_pause,
		(ParamFloat<px4::params::DYTG_SC_YSTP>) _param_scan_yaw_step,
		(ParamFloat<px4::params::DYTG_SC_DWEL>) _param_scan_dwell,
		(ParamInt<px4::params::DYTG_CTRMS>) _param_center_ms,
		(ParamInt<px4::params::DYTG_RTRYMS>) _param_retry_ms,
		(ParamFloat<px4::params::DYTG_DLY_MS>) _param_delay_ms,
		(ParamFloat<px4::params::DYTG_MAXAGE>) _param_max_age,
		(ParamFloat<px4::params::DYTG_MAXJIT>) _param_max_gap,
		(ParamFloat<px4::params::DYTG_HOFF_T>) _param_handoff_blend_time,
		(ParamFloat<px4::params::DYTG_INTDLY>) _param_intercept_delay,
		(ParamFloat<px4::params::DYTG_N_FOL>) _param_n_follow,
		(ParamFloat<px4::params::DYTG_V_FOL>) _param_v_follow,
		(ParamFloat<px4::params::DYTG_KA_FOL>) _param_ka_follow,
		(ParamFloat<px4::params::DYTG_KV_FOL>) _param_kv_follow,
		(ParamFloat<px4::params::DYTG_N_INT>) _param_n_intercept,
		(ParamFloat<px4::params::DYTG_V_INT>) _param_v_intercept,
		(ParamFloat<px4::params::DYTG_KA_INT>) _param_ka_intercept,
		(ParamFloat<px4::params::DYTG_KV_INT>) _param_kv_intercept,
		(ParamFloat<px4::params::DYTG_VMIN>) _param_vmin,
		(ParamFloat<px4::params::DYTG_MAXV>) _param_max_vel,
		(ParamFloat<px4::params::DYTG_MAXACC>) _param_max_acc,
		(ParamFloat<px4::params::DYTG_RNG_MIN>) _param_net_range_min,
		(ParamFloat<px4::params::DYTG_RNG_MAX>) _param_net_range_max,
		(ParamInt<px4::params::DYTG_FIRE_AUX>) _param_manual_fire_aux,
		(ParamInt<px4::params::DYTG_FIRE_BTN>) _param_manual_fire_btn,
		(ParamInt<px4::params::DYTG_FIRE_EN>) _param_net_release_enable,
		(ParamInt<px4::params::DYTG_HOLD_EN>) _param_net_hold_enable,
		(ParamFloat<px4::params::DYTG_STOP_D>) _param_net_stop_distance,
		(ParamFloat<px4::params::DYTG_STOP_V>) _param_net_stop_speed,
		(ParamFloat<px4::params::DYTG_STOP_ACC>) _param_net_stop_accel,
		(ParamInt<px4::params::DYTG_NET_EN>) _param_net_decel_enable,
		(ParamInt<px4::params::DYTG_NET_MS>) _param_net_decel_ms,
		(ParamFloat<px4::params::DYTG_NET_SC>) _param_net_decel_scale,
		(ParamFloat<px4::params::DYTG_NET_ACC>) _param_net_decel_accel,
		(ParamFloat<px4::params::DYTG_MAXYAWR>) _param_max_yaw_rate_deg,
		(ParamFloat<px4::params::DYTG_YAWLIM>) _param_yaw_limit_deg,
		(ParamFloat<px4::params::DYTG_MAXDZ>) _param_max_dz,
		(ParamFloat<px4::params::DYTG_ZSCALE>) _param_z_scale,
		(ParamInt<px4::params::DYTG_ZXY_EN>) _param_zxy_enable,
		(ParamFloat<px4::params::DYTG_ZXY_MIN>) _param_zxy_min_scale,
		(ParamFloat<px4::params::DYTG_ZXY_FULL>) _param_zxy_full_los,
		(ParamInt<px4::params::DYTG_XYOVR_EN>) _param_xy_overshoot_enable,
		(ParamFloat<px4::params::DYTG_XYOVR_Z>) _param_xy_overshoot_z,
		(ParamFloat<px4::params::DYTG_XYOVR_V>) _param_xy_overshoot_speed,
		(ParamFloat<px4::params::DYTG_XYOVR_MIN>) _param_xy_overshoot_min_scale,
		(ParamInt<px4::params::DYTG_XYROT_EN>) _param_xy_turn_rate_enable,
		(ParamFloat<px4::params::DYTG_XYROT_W>) _param_xy_turn_rate_full,
		(ParamFloat<px4::params::DYTG_XYROT_MIN>) _param_xy_turn_rate_min_scale,
		(ParamFloat<px4::params::DYTG_XYDB>) _param_xy_deadband,
		(ParamFloat<px4::params::DYTG_XYFULL>) _param_xy_full,
		(ParamFloat<px4::params::DYTG_YAWLOS>) _param_yaw_los_min,
		(ParamFloat<px4::params::DYTG_XYSLEW>) _param_xy_slew_rate,
		(ParamFloat<px4::params::DYTG_FCONE>) _param_front_cone_deg,
		(ParamFloat<px4::params::DYTG_LPF_A>) _param_lpf_alpha,
		(ParamFloat<px4::params::DYTG_PREDMAX>) _param_pred_max,
		(ParamInt<px4::params::DYTG_LXSIGN>) _param_los_x_sign,
		(ParamInt<px4::params::DYTG_LYSIGN>) _param_los_y_sign,
		(ParamInt<px4::params::DYTG_RSIGN>) _param_roll_sign,
		(ParamInt<px4::params::DYTG_PSIGN>) _param_pitch_sign,
		(ParamInt<px4::params::DYTG_YSIGN>) _param_yaw_sign,
		(ParamFloat<px4::params::DYTG_ROFF>) _param_roll_off_deg,
		(ParamFloat<px4::params::DYTG_POFF>) _param_pitch_off_deg,
		(ParamFloat<px4::params::DYTG_YOFF>) _param_yaw_off_deg,
		(ParamInt<px4::params::DYTG_MNT_EN>) _param_mount_enable,
		(ParamFloat<px4::params::DYTG_MNT_R>) _param_mount_roll_deg,
		(ParamFloat<px4::params::DYTG_MNT_P>) _param_mount_pitch_deg,
		(ParamFloat<px4::params::DYTG_MNT_Y>) _param_mount_yaw_deg,
		(ParamFloat<px4::params::DYT_HOME_YAW>) _param_home_yaw_deg,
		(ParamFloat<px4::params::DYT_HOME_PIT>) _param_home_pitch_deg
	);
};

DytGuidance::DytGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	memset(&_last_target, 0, sizeof(_last_target));
}

bool DytGuidance::init()
{
	ScheduleOnInterval(20_ms);
	return true;
}

int DytGuidance::print_status()
{
	show_status();
	return PX4_OK;
}

void DytGuidance::show_status()
{
	PX4_INFO("state: %u", static_cast<unsigned>(_state));
	PX4_INFO("handoff: coop_en=%ld geo_en=%ld active=%d controlling_vehicle=%d",
		 static_cast<long>(_param_coop_enable.get()),
		 static_cast<long>(_param_midcourse_geo_enable.get()), midcourse_handoff_active(), vehicle_control_active());
	PX4_INFO("midcourse target: id=%lu age=%.3f yaw=%.1f pitch=%.1f burst=%d",
		 static_cast<unsigned long>(_midcourse_target_info.mavid),
		 static_cast<double>(_last_midcourse_target_time > 0 ?
				     (hrt_absolute_time() - _last_midcourse_target_time) * 1e-6f : -1.f),
		 static_cast<double>(_midcourse_yaw_deg),
		 static_cast<double>(_midcourse_pitch_deg),
		 _midcourse_burst_remaining);
	Vector3f midcourse_target_position{};

	if (midcourse_target_position_local(midcourse_target_position)) {
		const Vector3f own_position(_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
		const Vector3f los_ned = midcourse_target_position - own_position;
		const float horizontal_distance = sqrtf(los_ned(0) * los_ned(0) + los_ned(1) * los_ned(1));
		const float target_up = -los_ned(2);
		const float elevation_deg = math::degrees(atan2f(target_up, math::max(horizontal_distance, 0.001f)));
		const float bearing_deg = math::degrees(atan2f(los_ned(1), los_ned(0)));
		float body_yaw_deg = NAN;
		float body_pitch_deg = NAN;
		(void)compute_midcourse_gimbal_angle(body_yaw_deg, body_pitch_deg);
		PX4_INFO("midcourse geometry: horiz=%.1f m target_up=%.1f m elev=%.1f deg bearing=%.1f deg",
			 static_cast<double>(horizontal_distance),
			 static_cast<double>(target_up),
			 static_cast<double>(elevation_deg),
			 static_cast<double>(bearing_deg));
		PX4_INFO("midcourse command angle: yaw=%.1f deg pitch=%.1f deg",
			 static_cast<double>(body_yaw_deg),
			 static_cast<double>(body_pitch_deg));
	}

	if (global_position_valid() && midcourse_target_geo_valid()) {
		const float target_command_alt = midcourse_target_command_alt();
		const float target_rel_alt = midcourse_target_relative_alt();
		const float own_rel_alt = own_midcourse_relative_alt();
		PX4_INFO("midcourse altitude: mode=%ld own_msl=%.1f m own_rel=%.1f m target_raw=%.1f m target_rel=%.1f m ref=%.1f m target_cmd=%.1f m off=%.1f m delta_cmd=%.1f m",
			 static_cast<long>(_param_midcourse_alt_mode.get()),
			 static_cast<double>(_vehicle_global_position.alt),
			 static_cast<double>(own_rel_alt),
			 static_cast<double>(_midcourse_target_info.alt),
			 static_cast<double>(target_rel_alt),
			 static_cast<double>(_midcourse_target_alt_ref_m),
			 static_cast<double>(target_command_alt),
			 static_cast<double>(_param_midcourse_target_alt_offset.get()),
			 static_cast<double>(target_command_alt - _vehicle_global_position.alt));
	}

	PX4_INFO("midcourse geotrack: active=%d ownship_age=%.3f target_tx_age=%.3f",
		 _midcourse_geotrack_active,
		 static_cast<double>(_last_midcourse_ownship_time > 0 ?
				     (hrt_absolute_time() - _last_midcourse_ownship_time) * 1e-6f : -1.f),
		 static_cast<double>(_last_midcourse_geo_target_time > 0 ?
				     (hrt_absolute_time() - _last_midcourse_geo_target_time) * 1e-6f : -1.f));
	PX4_INFO("target fresh: %d", target_fresh());
	PX4_INFO("target locked: %d", target_locked());
	PX4_INFO("target geometry: %d", target_geometry_valid());
	PX4_INFO("net decel: en=%ld low=%d active=%d init=%.2f target=%.2f",
		 static_cast<long>(_param_net_decel_enable.get()),
		 _net_decel_low_speed_active,
		 _net_decel_until > hrt_absolute_time(),
		 static_cast<double>(_net_decel_initial_speed),
		 static_cast<double>(_net_decel_target_speed));
	PX4_INFO("net fire: en=%ld range=%.2f m window=[%.2f, %.2f] m fresh=%d sent=%d",
		 static_cast<long>(_param_net_release_enable.get()),
		 static_cast<double>(_laser_distance_m),
		 static_cast<double>(_param_net_range_min.get()),
		 static_cast<double>(_param_net_range_max.get()),
		 _last_laser_sample_time > 0 && hrt_elapsed_time(&_last_laser_sample_time) <= 200_ms,
		 _net_release_sent);
	PX4_INFO("manual fire: aux=%ld value=%.2f btn=%ld request=%d",
		 static_cast<long>(_param_manual_fire_aux.get()),
		 static_cast<double>(aux_value(_param_manual_fire_aux.get())),
		 static_cast<long>(_param_manual_fire_btn.get()),
		 manual_fire_requested());
	PX4_INFO("net hold: en=%ld active=%d age=%.3f s",
		 static_cast<long>(_param_net_hold_enable.get()),
		 _net_hold_active,
		 static_cast<double>(_net_hold_start_time > 0 ?
				     (hrt_absolute_time() - _net_hold_start_time) * 1e-6f : -1.f));
	PX4_INFO("net brake: active=%d stop_d=%.2f stop_v=%.2f",
		 _net_brake_active,
		 static_cast<double>(_param_net_stop_distance.get()),
		 static_cast<double>(_param_net_stop_speed.get()));
	PX4_INFO("net brake acc: %.2f", static_cast<double>(_param_net_stop_accel.get()));
	PX4_INFO("observations: %d", _observation_count);
	PX4_INFO("payload lock seen: %d", _payload_lock_seen);
	PX4_INFO("payload lost hold: %d", _payload_lost_hold);
	PX4_INFO("preconditions ok: %d", preconditions_ok());
	PX4_INFO("activation request: %d", activation_requested());
	PX4_INFO("midcourse pointing request: %d", midcourse_pointing_requested());
	PX4_INFO("manual activation: %d", _manual_activation);
	PX4_INFO("activation aux value: %.2f", static_cast<double>(aux_value(_param_act_aux.get())));
	PX4_INFO("activation button: %ld buttons=0x%04x",
		 static_cast<long>(_param_act_btn.get()), static_cast<unsigned>(_manual_control.buttons));
	PX4_INFO("midcourse aux: %ld value: %.2f",
		 static_cast<long>(_param_midcourse_act_aux.get()),
		 static_cast<double>(aux_value(_param_midcourse_act_aux.get())));
	PX4_INFO("midcourse button: %ld buttons=0x%04x",
		 static_cast<long>(_param_midcourse_act_btn.get()), static_cast<unsigned>(_manual_control.buttons));
	PX4_INFO("payload switch: %u", static_cast<unsigned>(_manual_switches.payload_power_switch));
	PX4_INFO("auto activation: en=%ld streak=%d/%ld",
		 static_cast<long>(_param_auto_enable.get()), _auto_lock_streak, static_cast<long>(_param_auto_frames.get()));
	PX4_INFO("manual valid: %u roll=%.2f pitch=%.2f yaw=%.2f sticks=%u",
		 static_cast<unsigned>(_manual_control.valid),
		 static_cast<double>(_manual_control.roll),
		 static_cast<double>(_manual_control.pitch),
		 static_cast<double>(_manual_control.yaw),
		 static_cast<unsigned>(_manual_control.sticks_moving));
	PX4_INFO("activation aux: %ld", static_cast<long>(_param_act_aux.get()));
	PX4_INFO("intercept aux: %ld", static_cast<long>(_param_int_aux.get()));
	PX4_INFO("home angle: yaw=%.1f pitch=%.1f",
		 static_cast<double>(_param_home_yaw_deg.get()),
		 static_cast<double>(_param_home_pitch_deg.get()));
	PX4_INFO("midcourse mount rotation: en=%ld rpy=(%.1f %.1f %.1f) deg",
		 static_cast<long>(_param_mount_enable.get()),
		 static_cast<double>(_param_mount_roll_deg.get()),
		 static_cast<double>(_param_mount_pitch_deg.get()),
		 static_cast<double>(_param_mount_yaw_deg.get()));
	PX4_INFO("midcourse gimbal mode: %ld", static_cast<long>(_param_midcourse_gimbal_mode.get()));
	PX4_INFO("command pubs: %lu", static_cast<unsigned long>(_command_pub_count));
	PX4_INFO("last command: %u", static_cast<unsigned>(_last_command));
	PX4_INFO("last command age: %.3f s", static_cast<double>(_last_command_time > 0 ?
			(hrt_absolute_time() - _last_command_time) * 1e-6 : -1.0));
}

void DytGuidance::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();

		if (_param_net_decel_enable.get() <= 0) {
			clear_net_decel();
		}

		if (_param_net_hold_enable.get() <= 0) {
			clear_net_hold();
		}
	}
}

void DytGuidance::update_subscriptions()
{
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_global_position_sub.update(&_vehicle_global_position);
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_local_position_setpoint_sub.update(&_vehicle_local_position_setpoint);
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_airspeed_validated_sub.update(&_airspeed_validated);
	_manual_control_sub.update(&_manual_control);
	_manual_switches_sub.update(&_manual_switches);

	update_gripper_release_trigger(hrt_absolute_time());

	dyt_target_s target{};

	while (_dyt_target_sub.update(&target)) {
		_last_target = target;
		_have_target = true;
		handle_new_target(target);
	}

	follower_info_s info{};

	while (_follower_info_sub.update(&info)) {
		const int32_t target_id = _param_midcourse_target_id.get();
		const bool target_id_matches = target_id <= 0 || info.mavid == static_cast<uint32_t>(target_id);
		const bool not_self = !_vehicle_id_initialized || info.mavid != _vehicle_id;
		const bool real_position_source = info.source == follower_info_s::SOURCE_REAL_POSITION ||
						  info.source == follower_info_s::SOURCE_LEADER_REAL_POSITION;

		if (target_id_matches && not_self && real_position_source &&
		    PX4_ISFINITE(info.lat) && PX4_ISFINITE(info.lon) && PX4_ISFINITE(info.alt)) {
			const float target_alt = static_cast<float>(info.alt);

			if (_midcourse_target_alt_ref_mavid != info.mavid || !PX4_ISFINITE(_midcourse_target_alt_ref_m)) {
				_midcourse_target_alt_ref_mavid = info.mavid;
				_midcourse_target_alt_ref_m = target_alt;

			} else if (target_alt < _midcourse_target_alt_ref_m) {
				_midcourse_target_alt_ref_m = target_alt;
			}

			_midcourse_target_info = info;
			_last_midcourse_target_time = hrt_absolute_time();
			_midcourse_handoff_latched = true;
		}
	}
}

void DytGuidance::update_vehicle_id()
{
	if (_vehicle_id_initialized) {
		return;
	}

	int32_t mav_sys_id = 0;
	const param_t handle = param_find("MAV_SYS_ID");

	if (handle != PARAM_INVALID && param_get(handle, &mav_sys_id) == PX4_OK && mav_sys_id > 0) {
		_vehicle_id = static_cast<uint32_t>(mav_sys_id);
		_vehicle_id_initialized = true;
	}
}

float DytGuidance::aux_value(int index) const
{
	switch (index) {
	case 1: return _manual_control.aux1;
	case 2: return _manual_control.aux2;
	case 3: return _manual_control.aux3;
	case 4: return _manual_control.aux4;
	case 5: return _manual_control.aux5;
	case 6: return _manual_control.aux6;
	default: return NAN;
	}
}

bool DytGuidance::aux_switch_active(int index) const
{
	const float value = aux_value(index);
	return PX4_ISFINITE(value) && value > 0.5f;
}

bool DytGuidance::button_active(int button) const
{
	if (button < 0 || button > 15) {
		return false;
	}

	return (_manual_control.buttons & (1u << button)) != 0;
}

bool DytGuidance::payload_switch_active() const
{
	return _manual_switches.payload_power_switch == manual_control_switches_s::SWITCH_POS_ON;
}

bool DytGuidance::activation_requested() const
{
	return _manual_activation || aux_switch_active(_param_act_aux.get()) || button_active(_param_act_btn.get())
	       || payload_switch_active();
}

bool DytGuidance::midcourse_pointing_requested() const
{
	if (_param_coop_enable.get() <= 0) {
		return false;
	}

	const int act_aux = _param_midcourse_act_aux.get();
	const int act_btn = _param_midcourse_act_btn.get();

	return (act_aux == 0 && act_btn < 0) || aux_switch_active(act_aux) || button_active(act_btn);
}

bool DytGuidance::manual_fire_requested() const
{
	return aux_switch_active(_param_manual_fire_aux.get()) || button_active(_param_manual_fire_btn.get());
}

bool DytGuidance::preconditions_ok() const
{
	return _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && !_vehicle_status.failsafe
	       && _vehicle_local_position.xy_valid
	       && _vehicle_local_position.z_valid;
}

bool DytGuidance::manual_takeover_detected() const
{
	if (!_manual_control.valid) {
		return false;
	}

	const float threshold = _param_stick_takeover.get();
	return fabsf(_manual_control.roll) > threshold
	       || fabsf(_manual_control.pitch) > threshold
	       || fabsf(_manual_control.yaw) > threshold;
}

void DytGuidance::handle_new_target(const dyt_target_s &target)
{
	if (!target.target_valid || !target_geometry_valid(target)) {
		return;
	}

	Vector3f los_body;

	if (!build_los_body(target, los_body)) {
		return;
	}

	push_observation(los_body, target.timestamp_sample, target.frame_dt_s);
}

bool DytGuidance::build_los_body(const dyt_target_s &target, Vector3f &los_body) const
{
	const float los_x = target.los_x_rad * static_cast<float>(_param_los_x_sign.get());
	const float los_y = target.los_y_rad * static_cast<float>(_param_los_y_sign.get());

	Vector3f los_gimbal(1.f, tanf(los_x), tanf(los_y));

	if (los_gimbal.norm_squared() < 1e-6f) {
		return false;
	}

	los_gimbal.normalize();

	const float roll = target.gimbal_roll_rad * static_cast<float>(_param_roll_sign.get())
			   + math::radians(_param_roll_off_deg.get());
	const float pitch = target.gimbal_pitch_frame_rad * static_cast<float>(_param_pitch_sign.get())
			    + math::radians(_param_pitch_off_deg.get());
	const float yaw = target.gimbal_yaw_rad * static_cast<float>(_param_yaw_sign.get())
			  + math::radians(_param_yaw_off_deg.get());

	const Dcmf gimbal_to_mount(Eulerf(roll, pitch, yaw));
	const Vector3f los_mount = gimbal_to_mount * los_gimbal;
	los_body = rotate_mount_los_to_body(los_mount);

	if (!PX4_ISFINITE(los_body(0)) || !PX4_ISFINITE(los_body(1)) || !PX4_ISFINITE(los_body(2))
	    || los_body.norm_squared() < 1e-6f) {
		return false;
	}

	los_body.normalize();
	return true;
}

void DytGuidance::push_observation(const Vector3f &los_body, hrt_abstime sample_time, float frame_dt_s)
{
	if (_observation_count < OBS_BUFFER_LEN) {
		_observations[_observation_count++] = {sample_time, los_body, frame_dt_s};

	} else {
		for (int i = 1; i < OBS_BUFFER_LEN; ++i) {
			_observations[i - 1] = _observations[i];
		}

		_observations[OBS_BUFFER_LEN - 1] = {sample_time, los_body, frame_dt_s};
	}

	_los_body_latest = los_body;
}

void DytGuidance::clear_observations()
{
	_observation_count = 0;

	for (auto &observation : _observations) {
		observation = LosObservation{};
	}

	_los_filtered.zero();
	_prev_los_filtered.zero();
	_omega_los.zero();
	_los_ned.zero();
	_los_filter_initialized = false;
	_prev_los_update = 0;
	_last_track_setpoint_time = 0;
}

bool DytGuidance::update_los_estimate(hrt_abstime now)
{
	if (_observation_count <= 0) {
		return false;
	}

	Vector3f los_body = _observations[_observation_count - 1].los_body;

	if (_observation_count >= 2) {
		const LosObservation &latest = _observations[_observation_count - 1];
		const LosObservation &previous = _observations[_observation_count - 2];
		const float dt_obs = (latest.sample_time - previous.sample_time) * 1e-6f;

		if (dt_obs > 0.002f) {
			const Vector3f du_body = (latest.los_body - previous.los_body) / dt_obs;
			const float dt_pred = math::constrain((now - latest.sample_time) * 1e-6f, 0.f, _param_pred_max.get());
			los_body = latest.los_body + du_body * dt_pred;
		}
	}

	if (!PX4_ISFINITE(los_body(0)) || !PX4_ISFINITE(los_body(1)) || !PX4_ISFINITE(los_body(2))
	    || los_body.norm_squared() < 1e-6f) {
		return false;
	}

	los_body.normalize();

	const Dcmf body_to_ned(Quatf(_vehicle_attitude.q));
	const Vector3f los_raw = body_to_ned * los_body;

	if (!_los_filter_initialized) {
		_los_filtered = los_raw;
		_prev_los_filtered = los_raw;
		_prev_los_update = now;
		_omega_los.zero();
		_los_filter_initialized = true;

	} else {
		const float alpha = math::constrain(_param_lpf_alpha.get(), 0.f, 0.99f);
		_los_filtered = _los_filtered * alpha + los_raw * (1.f - alpha);

		if (_los_filtered.norm_squared() > 1e-6f) {
			_los_filtered.normalize();
		}

		const float dt = (now - _prev_los_update) * 1e-6f;

		if (dt > 0.005f) {
			const Vector3f du_dt = (_los_filtered - _prev_los_filtered) / dt;
			_omega_los = _los_filtered.cross(du_dt);
			_prev_los_filtered = _los_filtered;
			_prev_los_update = now;
		}
	}

	_los_ned = _los_filtered;
	return true;
}

void DytGuidance::capture_hold_setpoint()
{
	_hold_position(0) = _vehicle_local_position.x;
	_hold_position(1) = _vehicle_local_position.y;
	_hold_position(2) = _vehicle_local_position.z;
	_hold_yaw = Eulerf(Quatf(_vehicle_attitude.q)).psi();
}

bool DytGuidance::target_locked() const
{
	// Lock state is reported by the payload independently of whether its pixel miss
	// distance has a calibrated conversion to angular LOS.
	return _have_target && _last_target.tracking_state == dyt_target_s::TRACKING_STATE_LOCKED;
}

bool DytGuidance::target_geometry_valid(const dyt_target_s &target) const
{
	const bool bbox_valid = PX4_ISFINITE(target.bbox_width_px) && PX4_ISFINITE(target.bbox_height_px)
				&& target.bbox_width_px >= TARGET_MIN_BBOX_PX
				&& target.bbox_height_px >= TARGET_MIN_BBOX_PX;
	const bool los_valid = PX4_ISFINITE(target.los_x_rad) && PX4_ISFINITE(target.los_y_rad)
			       && fabsf(target.los_x_rad) <= TARGET_MAX_LOS_RAD
			       && fabsf(target.los_y_rad) <= TARGET_MAX_LOS_RAD;
	const bool yaw_valid = PX4_ISFINITE(target.gimbal_yaw_rad)
			       && fabsf(target.gimbal_yaw_rad) <= TARGET_MAX_GIMBAL_YAW_RAD;
	const bool locked = target.tracking_state == dyt_target_s::TRACKING_STATE_LOCKED && target.target_valid;

	return los_valid && yaw_valid && (locked || bbox_valid);
}

bool DytGuidance::target_geometry_valid() const
{
	return _have_target && target_geometry_valid(_last_target);
}

bool DytGuidance::target_hint_detected() const
{
	const bool bbox_valid = PX4_ISFINITE(_last_target.bbox_width_px) && PX4_ISFINITE(_last_target.bbox_height_px)
				&& _last_target.bbox_width_px >= TARGET_MIN_BBOX_PX
				&& _last_target.bbox_height_px >= TARGET_MIN_BBOX_PX;

	return _have_target && _last_target.auto_hint && target_fresh() && bbox_valid;
}

bool DytGuidance::target_fresh() const
{
	if (!_have_target || _last_target.timestamp_sample == 0) {
		return false;
	}

	const float age_s = (hrt_absolute_time() - _last_target.timestamp_sample) * 1e-6f;
	const bool age_ok = age_s <= _param_max_age.get();
	const bool gap_ok = !_have_target || (_last_target.frame_dt_s <= 0.f) || (_last_target.frame_dt_s <= _param_max_gap.get());

	return age_ok && gap_ok && _last_target.tracking_state != dyt_target_s::TRACKING_STATE_TIMEOUT
	       && _last_target.tracking_state != dyt_target_s::TRACKING_STATE_ERROR;
}

bool DytGuidance::target_lock_candidate() const
{
	if (!_have_target || _last_target.timestamp_sample == 0 || !target_fresh()) {
		return false;
	}

	const float age_s = (hrt_absolute_time() - _last_target.timestamp_sample) * 1e-6f;

	if (age_s > 0.3f) {
		return false;
	}

	if (_last_target.tracking_state == dyt_target_s::TRACKING_STATE_TIMEOUT ||
	    _last_target.tracking_state == dyt_target_s::TRACKING_STATE_ERROR) {
		return false;
	}

	if (!target_geometry_valid(_last_target)) {
		return false;
	}

	if (target_locked()) {
		return true;
	}

	if (!_last_target.auto_hint) {
		return false;
	}

	return fabsf(_last_target.los_x_rad) <= TARGET_MAX_HINT_LOS_RAD
	       && fabsf(_last_target.los_y_rad) <= TARGET_MAX_HINT_LOS_RAD;
}

bool DytGuidance::target_usable() const
{
	return target_locked() && target_fresh() && target_geometry_valid() && _observation_count > 0;
}

bool DytGuidance::intercept_allowed() const
{
	if (!target_usable()) {
		return false;
	}

	if ((_param_delay_ms.get() * 1e-3f) > _param_intercept_delay.get()) {
		return false;
	}

	if (_last_target.frame_dt_s > _param_max_gap.get()) {
		return false;
	}

	const float cone_cos = cosf(math::radians(_param_front_cone_deg.get()));
	return _los_body_latest(0) > cone_cos;
}

void DytGuidance::update_net_release_trigger(hrt_abstime now)
{
	const bool manual_fire_request = manual_fire_requested();
	const bool manual_fire_rising = manual_fire_request && !_prev_manual_fire_request;
	_prev_manual_fire_request = manual_fire_request;

	if (!target_usable() || _net_release_sent) {
		return;
	}

	if (manual_fire_rising) {
		send_net_release_command(now);
		_net_release_sent = true;
		return;
	}

	const float range_min = _param_net_range_min.get();
	const float range_max = _param_net_range_max.get();
	const bool range_window_valid = PX4_ISFINITE(range_min) && PX4_ISFINITE(range_max)
					&& range_min >= 0.05f && range_max > range_min;

	if (_param_net_release_enable.get() <= 0 || !range_window_valid || !update_laser_distance(now)) {
		return;
	}

	if (_laser_distance_m >= range_min && _laser_distance_m <= range_max) {
		send_net_release_command(now);
		_net_release_sent = true;
	}
}

bool DytGuidance::update_laser_distance(hrt_abstime now)
{
	hrt_abstime newest_timestamp{0};
	float newest_distance_m{NAN};

	for (unsigned i = 0; i < _distance_sensor_subs.size(); ++i) {
		distance_sensor_s distance{};

		if (!_distance_sensor_subs[i].update(&distance)) {
			continue;
		}

		const bool fresh = distance.timestamp > 0 && distance.timestamp <= now
				   && now - distance.timestamp <= 200_ms;
		device::Device::DeviceId device_id{};
		device_id.devid = distance.device_id;
		const bool valid_measurement = device_id.devid_s.devtype == DRV_DIST_DEVTYPE_SDM50
					       && distance.type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER
					       && distance.orientation == distance_sensor_s::ROTATION_FORWARD_FACING
					       && distance.signal_quality > 0
					       && PX4_ISFINITE(distance.current_distance)
					       && PX4_ISFINITE(distance.min_distance)
					       && PX4_ISFINITE(distance.max_distance)
					       && distance.current_distance >= distance.min_distance
					       && distance.current_distance <= distance.max_distance;

		if (fresh && valid_measurement && distance.timestamp > newest_timestamp) {
			newest_timestamp = distance.timestamp;
			newest_distance_m = distance.current_distance;
		}
	}

	if (newest_timestamp == 0 || newest_timestamp == _last_laser_sample_time) {
		if (_last_laser_sample_time == 0 || now < _last_laser_sample_time
		    || now - _last_laser_sample_time > 200_ms) {
			_laser_distance_m = NAN;
		}

		return false;
	}

	_last_laser_sample_time = newest_timestamp;
	_laser_distance_m = newest_distance_m;
	return true;
}

void DytGuidance::send_net_release_command(hrt_abstime now)
{
	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
	cmd.param1 = 1.f;
	cmd.param2 = static_cast<float>(vehicle_command_s::GRIPPER_ACTION_RELEASE);
	cmd.target_system = _vehicle_status.system_id;
	cmd.target_component = _vehicle_status.component_id;
	cmd.source_system = _vehicle_status.system_id;
	cmd.source_component = _vehicle_status.component_id;
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);
	start_net_hold_after_release();
}

void DytGuidance::clear_net_release_trigger()
{
	_last_laser_sample_time = 0;
	_laser_distance_m = NAN;
}

void DytGuidance::update_gripper_release_trigger(hrt_abstime now)
{
	gripper_s gripper{};

	while (_gripper_sub.update(&gripper)) {
		if (gripper.command != gripper_s::COMMAND_RELEASE) {
			continue;
		}

		const int32_t duration_ms = math::constrain(_param_net_decel_ms.get(), int32_t{0}, int32_t{5000});
		const bool enabled = _param_net_decel_enable.get() > 0 && duration_ms > 0
				     && _param_net_decel_scale.get() < 0.99f
				     && _param_net_decel_accel.get() > 0.01f;

		if (!enabled) {
			clear_net_decel();
			continue;
		}

			_net_decel_pending = true;
			start_net_hold_after_release();
		}

	if (_net_decel_pending) {
		Vector3f vehicle_velocity(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

			if (!PX4_ISFINITE(vehicle_velocity(0)) || !PX4_ISFINITE(vehicle_velocity(1)) || !PX4_ISFINITE(vehicle_velocity(2))) {
				vehicle_velocity.zero();
			}

			start_net_decel_if_ready(now, vehicle_velocity);
		}
	}

void DytGuidance::start_net_hold_after_release()
{
	if (_param_net_hold_enable.get() <= 0 || !vehicle_control_active() || !preconditions_ok()) {
		return;
	}

	_net_hold_active = true;
	_net_brake_active = true;
	_net_hold_start_time = hrt_absolute_time();
	clear_net_decel();
}

void DytGuidance::clear_net_hold()
{
	_net_hold_active = false;
	_net_brake_active = false;
	_net_hold_start_time = 0;
}

void DytGuidance::publish_net_brake_setpoint()
{
	const hrt_abstime now = hrt_absolute_time();
	Vector2f vehicle_velocity_xy(_vehicle_local_position.vx, _vehicle_local_position.vy);
	float speed = vehicle_velocity_xy.norm();

	if (!PX4_ISFINITE(speed) || speed <= math::constrain(_param_net_stop_speed.get(), 0.05f, 1.f)) {
		capture_hold_setpoint();
		_net_brake_active = false;
		publish_hold_setpoint();
		return;
	}

	const float stop_distance = math::constrain(_param_net_stop_distance.get(), 0.1f, 10.f);
	const float max_brake_accel = math::min(math::max(_param_net_stop_accel.get(), 0.1f), math::max(_param_max_acc.get(), 0.5f));
	const float required_accel = speed * speed / (2.f * stop_distance);
	const float brake_accel = math::constrain(required_accel, 0.1f, max_brake_accel);
	const Vector2f brake_xy = -vehicle_velocity_xy / speed * brake_accel;

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.position[0] = NAN;
	setpoint.position[1] = NAN;
	setpoint.position[2] = _vehicle_local_position.z;
	setpoint.velocity[0] = 0.f;
	setpoint.velocity[1] = 0.f;
	setpoint.velocity[2] = 0.f;
	setpoint.acceleration[0] = brake_xy(0);
	setpoint.acceleration[1] = brake_xy(1);
	setpoint.acceleration[2] = 0.f;
	setpoint.yaw = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : Eulerf(Quatf(_vehicle_attitude.q)).psi();
	setpoint.yawspeed = 0.f;

	_velocity_sp.zero();
	_acceleration_sp = Vector3f(brake_xy(0), brake_xy(1), 0.f);
	_yaw_sp = setpoint.yaw;
	_yaw_rate_sp = 0.f;
	_last_track_setpoint_time = now;

	_trajectory_setpoint_pub.publish(setpoint);
}

void DytGuidance::start_net_decel_if_ready(hrt_abstime now, const Vector3f &vehicle_velocity)
{
	if (!_net_decel_pending) {
		return;
	}

	const int32_t duration_ms = math::constrain(_param_net_decel_ms.get(), int32_t{0}, int32_t{5000});
	const bool enabled = _param_net_decel_enable.get() > 0 && duration_ms > 0
			     && _param_net_decel_scale.get() < 0.99f
			     && _param_net_decel_accel.get() > 0.01f;

	if (!enabled) {
		clear_net_decel();
		return;
	}

	const Vector2f vehicle_velocity_xy(vehicle_velocity(0), vehicle_velocity(1));
	float initial_speed = vehicle_velocity_xy.norm();

	if (!PX4_ISFINITE(initial_speed) || initial_speed < 0.1f) {
		const Vector2f velocity_sp_xy(_velocity_sp(0), _velocity_sp(1));
		initial_speed = velocity_sp_xy.norm();
	}

	if (!PX4_ISFINITE(initial_speed) || initial_speed < 0.1f) {
		return;
	}

	const float speed_scale = math::constrain(_param_net_decel_scale.get(), 0.f, 1.f);
	_net_decel_initial_speed = initial_speed;
	_net_decel_target_speed = initial_speed * speed_scale;
	_net_decel_until = now + static_cast<hrt_abstime>(duration_ms) * 1000ULL;
	_net_decel_low_speed_active = true;
	_net_decel_pending = false;
}

void DytGuidance::clear_net_decel()
{
	_net_decel_until = 0;
	_net_decel_pending = false;
	_net_decel_low_speed_active = false;
	_net_decel_initial_speed = 0.f;
	_net_decel_target_speed = 0.f;
}

void DytGuidance::apply_net_decel_velocity_scale(Vector2f &vel_xy) const
{
	if (!_net_decel_low_speed_active || _param_net_decel_enable.get() <= 0) {
		return;
	}

	const float speed_scale = math::constrain(_param_net_decel_scale.get(), 0.f, 1.f);
	vel_xy *= speed_scale;
}

Vector3f DytGuidance::net_decel_accel_ned(hrt_abstime now, const Vector3f &vehicle_velocity)
{
	if (_net_decel_until == 0 || _param_net_decel_enable.get() <= 0) {
		return Vector3f{};
	}

	const Vector2f vehicle_velocity_xy(vehicle_velocity(0), vehicle_velocity(1));
	const float speed = vehicle_velocity_xy.norm();

	if (!PX4_ISFINITE(speed) || speed <= math::max(_net_decel_target_speed, 0.f) + 0.05f) {
		_net_decel_until = 0;
		return Vector3f{};
	}

	if (now >= _net_decel_until) {
		_net_decel_until = 0;
		return Vector3f{};
	}

	const hrt_abstime remaining_us = _net_decel_until - now;
	const float remaining_s = math::max(remaining_us * 1e-6f, 0.02f);
	const float requested_decel = (speed - math::max(_net_decel_target_speed, 0.f)) / remaining_s;
	const float decel = math::constrain(requested_decel, 0.f, math::max(_param_net_decel_accel.get(), 0.1f));

	if (decel <= 0.01f) {
		return Vector3f{};
	}

	const Vector2f brake_xy = -vehicle_velocity_xy / speed * decel;
	return Vector3f(brake_xy(0), brake_xy(1), 0.f);
}

bool DytGuidance::midcourse_handoff_active() const
{
	if (_param_coop_enable.get() > 0) {
		return true;
	}

	if (_midcourse_handoff_latched) {
		return true;
	}

	Vector3f target_position{};
	return midcourse_target_position_local(target_position);
}

bool DytGuidance::vehicle_control_active() const
{
	// States where the seeker would command aircraft motion (trajectory/offboard setpoints).
	const bool tracking = (_state == TaskState::TrackFollow || _state == TaskState::TrackIntercept);

	if (midcourse_handoff_active()) {
		// During midcourse/terminal handoff, the seeker only drives aircraft motion
		// after visual lock. Before lock it commands the gimbal only, while the
		// position-sharing follower keeps flying and prevents a hover break.
		return tracking;
	}

	// Standalone: the seeker owns aircraft motion in every active state (hold while searching).
	return _state != TaskState::Idle && _state != TaskState::Abort;
}

DytGuidance::TrackProfile DytGuidance::follow_profile() const
{
	return {_param_n_follow.get(), _param_v_follow.get(), _param_ka_follow.get(), _param_kv_follow.get(),
		dyt_guidance_status_s::SUBMODE_FOLLOW};
}

DytGuidance::TrackProfile DytGuidance::intercept_profile() const
{
	return {_param_n_intercept.get(), _param_v_intercept.get(), _param_ka_intercept.get(), _param_kv_intercept.get(),
		dyt_guidance_status_s::SUBMODE_INTERCEPT};
}

void DytGuidance::publish_hold_setpoint()
{
	_last_track_setpoint_time = 0;

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();
	_hold_position.copyTo(setpoint.position);
	setpoint.velocity[0] = 0.f;
	setpoint.velocity[1] = 0.f;
	setpoint.velocity[2] = 0.f;
	setpoint.acceleration[0] = NAN;
	setpoint.acceleration[1] = NAN;
	setpoint.acceleration[2] = NAN;
	setpoint.yaw = _hold_yaw;
	setpoint.yawspeed = NAN;

	_velocity_sp.zero();
	_acceleration_sp.zero();
	_yaw_sp = _hold_yaw;
	_yaw_rate_sp = NAN;

	_trajectory_setpoint_pub.publish(setpoint);
}

void DytGuidance::publish_track_setpoint(const TrackProfile &profile)
{
	const hrt_abstime now = hrt_absolute_time();
	update_net_release_trigger(now);

	if (!update_los_estimate(now)) {
		publish_hold_setpoint();
		return;
	}

	Vector3f vehicle_velocity(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);

	if (!PX4_ISFINITE(vehicle_velocity(0)) || !PX4_ISFINITE(vehicle_velocity(1)) || !PX4_ISFINITE(vehicle_velocity(2))) {
		vehicle_velocity.zero();
	}

	start_net_decel_if_ready(now, vehicle_velocity);

	Vector2f horizontal_los(_los_ned(0), _los_ned(1));
	const float los_xy_norm = horizontal_los.norm();
	const float xy_deadband = math::constrain(_param_xy_deadband.get(), 0.f, 0.8f);
	const float xy_full = math::constrain(_param_xy_full.get(), xy_deadband + 0.05f, 1.f);
	float xy_scale{0.f};

	if (los_xy_norm > xy_deadband) {
		xy_scale = math::constrain((los_xy_norm - xy_deadband) / (xy_full - xy_deadband), 0.f, 1.f);
		xy_scale = xy_scale * xy_scale * (3.f - 2.f * xy_scale);
	}

	float zxy_scale{1.f};

	if (_param_zxy_enable.get() > 0) {
		const float zxy_min = math::constrain(_param_zxy_min_scale.get(), 0.f, 1.f);
		const float zxy_full = math::constrain(_param_zxy_full_los.get(), 0.05f, 1.f);
		float z_ratio = math::constrain(fabsf(_los_ned(2)) / zxy_full, 0.f, 1.f);
		z_ratio = z_ratio * z_ratio * (3.f - 2.f * z_ratio);
		zxy_scale = 1.f - (1.f - zxy_min) * z_ratio;
	}

	Vector2f vel_xy_raw(0.f, 0.f);
	Vector2f horizontal_dir(0.f, 0.f);

	if (los_xy_norm > 1e-3f) {
		horizontal_dir = horizontal_los / los_xy_norm;
	}

	float xy_overshoot_scale{1.f};
	float xy_turn_rate_scale{1.f};

	if (_param_xy_overshoot_enable.get() > 0 && los_xy_norm > 1e-3f) {
		const Vector2f vehicle_velocity_xy(vehicle_velocity(0), vehicle_velocity(1));
		const float horizontal_closing_speed = vehicle_velocity_xy.dot(horizontal_dir);

		if (horizontal_closing_speed < 0.f) {
			const float overshoot_min = math::constrain(_param_xy_overshoot_min_scale.get(), 0.f, 1.f);
			const float overshoot_z = math::constrain(_param_xy_overshoot_z.get(), 0.f, 0.95f);
			const float overshoot_speed = math::max(_param_xy_overshoot_speed.get(), 0.1f);
			float z_ratio = math::constrain((fabsf(_los_ned(2)) - overshoot_z) / (1.f - overshoot_z), 0.f, 1.f);
			float reverse_ratio = math::constrain(-horizontal_closing_speed / overshoot_speed, 0.f, 1.f);
			z_ratio = z_ratio * z_ratio * (3.f - 2.f * z_ratio);
			reverse_ratio = reverse_ratio * reverse_ratio * (3.f - 2.f * reverse_ratio);
			const float guard_ratio = z_ratio * reverse_ratio;
			xy_overshoot_scale = 1.f - (1.f - overshoot_min) * guard_ratio;
		}
	}

	if (_param_xy_turn_rate_enable.get() > 0 && los_xy_norm > 1e-3f) {
		const float turn_rate_min = math::constrain(_param_xy_turn_rate_min_scale.get(), 0.f, 1.f);
		const float turn_rate_full = math::max(_param_xy_turn_rate_full.get(), 0.1f);
		const float overshoot_z = math::constrain(_param_xy_overshoot_z.get(), 0.f, 0.95f);
		const float los_xy_sq = math::max(los_xy_norm * los_xy_norm, 0.05f);
		float z_ratio = math::constrain((fabsf(_los_ned(2)) - overshoot_z) / (1.f - overshoot_z), 0.f, 1.f);
		float turn_ratio = math::constrain(fabsf(_omega_los(2)) / los_xy_sq / turn_rate_full, 0.f, 1.f);
		z_ratio = z_ratio * z_ratio * (3.f - 2.f * z_ratio);
		turn_ratio = turn_ratio * turn_ratio * (3.f - 2.f * turn_ratio);
		const float guard_ratio = z_ratio * turn_ratio;
		xy_turn_rate_scale = 1.f - (1.f - turn_rate_min) * guard_ratio;
	}

	const float xy_guard_scale = xy_overshoot_scale * xy_turn_rate_scale;

	if (los_xy_norm > 1e-3f) {
		vel_xy_raw = horizontal_dir * profile.v_cmd * xy_scale * zxy_scale * xy_guard_scale;
	}

	const float max_vel = math::max(_param_max_vel.get(), 0.1f);

	if (vel_xy_raw.norm() > max_vel) {
		vel_xy_raw = vel_xy_raw.normalized() * max_vel;
	}

	apply_net_decel_velocity_scale(vel_xy_raw);

	Vector2f vel_xy(vel_xy_raw);
	Vector2f previous_vel_xy(_velocity_sp(0), _velocity_sp(1));

	if (!PX4_ISFINITE(previous_vel_xy(0)) || !PX4_ISFINITE(previous_vel_xy(1))) {
		previous_vel_xy.zero();
	}

	const float dt_sp = _last_track_setpoint_time > 0 ? math::constrain((now - _last_track_setpoint_time) * 1e-6f,
			    0.005f, 0.1f) : 0.02f;
	const float max_delta_xy = math::max(_param_xy_slew_rate.get(), 0.1f) * dt_sp;
	const Vector2f delta_vel_xy = vel_xy_raw - previous_vel_xy;

	if (delta_vel_xy.norm() > max_delta_xy) {
		vel_xy = previous_vel_xy + delta_vel_xy.normalized() * max_delta_xy;
	}

	const float handoff_blend = track_handoff_blend(now);

	if (handoff_blend < 1.f) {
		const Vector2f handoff_vel_xy(_track_handoff_velocity(0), _track_handoff_velocity(1));
		vel_xy = handoff_vel_xy * (1.f - handoff_blend) + vel_xy * handoff_blend;
	}

	_velocity_sp(0) = vel_xy(0);
	_velocity_sp(1) = vel_xy(1);
	const float z_scale = math::constrain(_param_z_scale.get(), 0.f, 1.f);
	const float max_dz = math::max(_param_max_dz.get(), 0.1f);
	float velocity_sp_z = math::constrain(_los_ned(2) * profile.v_cmd * z_scale, -max_dz, max_dz);

	if (handoff_blend < 1.f) {
		velocity_sp_z = _track_handoff_velocity(2) * (1.f - handoff_blend) + velocity_sp_z * handoff_blend;
	}

	_velocity_sp(2) = velocity_sp_z;
	_last_track_setpoint_time = now;

	Vector3f horizontal_vehicle_velocity(vehicle_velocity(0), vehicle_velocity(1), 0.f);
	Vector3f horizontal_velocity_sp(_velocity_sp(0), _velocity_sp(1), 0.f);

	const float closing_proxy = math::max(_param_vmin.get(), horizontal_vehicle_velocity.dot(_los_ned));
	const Vector3f velocity_error = horizontal_velocity_sp - horizontal_vehicle_velocity;

	Vector3f pn_acc = profile.nav_gain * closing_proxy * (_omega_los.cross(_los_ned));
	Vector3f los_acc = profile.k_a * _los_ned;
	const Vector3f damp_acc = profile.k_v * velocity_error;

	pn_acc(0) *= xy_scale;
	pn_acc(1) *= xy_scale;
	los_acc(0) *= xy_scale;
	los_acc(1) *= xy_scale;
	pn_acc(0) *= zxy_scale;
	pn_acc(1) *= zxy_scale;
	los_acc(0) *= zxy_scale;
	los_acc(1) *= zxy_scale;
	pn_acc(0) *= xy_guard_scale;
	pn_acc(1) *= xy_guard_scale;
	los_acc(0) *= xy_guard_scale;
	los_acc(1) *= xy_guard_scale;

	_acceleration_sp = pn_acc + los_acc + damp_acc;
	_acceleration_sp += net_decel_accel_ned(now, vehicle_velocity);

	Vector2f acc_xy(_acceleration_sp(0), _acceleration_sp(1));

	if (acc_xy.norm() > _param_max_acc.get()) {
		acc_xy = acc_xy.normalized() * _param_max_acc.get();
		_acceleration_sp(0) = acc_xy(0);
		_acceleration_sp(1) = acc_xy(1);
	}

	_acceleration_sp(2) = 0.f;

	const float yaw_los_min = math::constrain(_param_yaw_los_min.get(), 0.01f, 1.f);
	const float current_yaw = Eulerf(Quatf(_vehicle_attitude.q)).psi();

	if (los_xy_norm <= yaw_los_min) {
		_yaw_sp = current_yaw;
		_yaw_rate_sp = 0.f;
	} else {
		const float desired_yaw = atan2f(_los_ned(1), _los_ned(0));
		const float yaw_error = matrix::wrap_pi(desired_yaw - current_yaw);
		const float yaw_limit = math::radians(_param_yaw_limit_deg.get());
		_yaw_sp = matrix::wrap_pi(current_yaw + math::constrain(yaw_error, -yaw_limit, yaw_limit));
		_yaw_rate_sp = math::constrain(yaw_error * 2.f,
					       -math::radians(_param_max_yaw_rate_deg.get()),
					       math::radians(_param_max_yaw_rate_deg.get()));
	}

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.position[0] = NAN;
	setpoint.position[1] = NAN;
	setpoint.position[2] = NAN;
	_velocity_sp.copyTo(setpoint.velocity);
	_acceleration_sp.copyTo(setpoint.acceleration);
	setpoint.yaw = _yaw_sp;
	setpoint.yawspeed = _yaw_rate_sp;

	_trajectory_setpoint_pub.publish(setpoint);
}

void DytGuidance::publish_offboard_mode(bool position_mode)
{
	offboard_control_mode_s mode{};
	mode.timestamp = hrt_absolute_time();
	mode.position = position_mode;
	mode.velocity = !position_mode;
	mode.acceleration = false;
	mode.attitude = false;
	mode.body_rate = false;
	mode.thrust_and_torque = false;
	mode.direct_actuator = false;
	_offboard_control_mode_pub.publish(mode);
}

void DytGuidance::request_offboard_mode()
{
	const hrt_abstime now = hrt_absolute_time();

	if ((now - _last_offboard_request) < 1_s) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.0f;
	cmd.param2 = 6.0f;
	cmd.target_system = _vehicle_status.system_id;
	cmd.target_component = _vehicle_status.component_id;
	cmd.source_system = _vehicle_status.system_id;
	cmd.source_component = _vehicle_status.component_id;
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);

	_last_offboard_request = now;
}

void DytGuidance::publish_status()
{
	dyt_guidance_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = static_cast<uint8_t>(_state);
	status.requested_submode = _requested_submode;

	if (_state == TaskState::TrackIntercept) {
		status.active_submode = dyt_guidance_status_s::SUBMODE_INTERCEPT;
	} else {
		status.active_submode = dyt_guidance_status_s::SUBMODE_FOLLOW;
	}

	status.lost_reason = _lost_reason;
	status.active = _state != TaskState::Idle && _state != TaskState::Abort;
	status.controlling_vehicle = vehicle_control_active();
	status.target_locked = target_locked();
	status.target_fresh = target_fresh();
	status.intercept_allowed = intercept_allowed();
	status.los_age_s = _have_target && _last_target.timestamp_sample > 0
			   ? (hrt_absolute_time() - _last_target.timestamp_sample) * 1e-6f : NAN;
	status.frame_dt_s = _have_target ? _last_target.frame_dt_s : NAN;
	status.delay_s = _param_delay_ms.get() * 1e-3f;
	_los_ned.copyTo(status.los_ned);
	_omega_los.copyTo(status.omega_los_ned);
	_velocity_sp.copyTo(status.velocity_sp);
	_acceleration_sp.copyTo(status.acceleration_sp);
	status.yaw_sp = _yaw_sp;
	status.yaw_rate_sp = _yaw_rate_sp;

	_dyt_guidance_status_pub.publish(status);
}

void DytGuidance::capture_track_handoff_velocity()
{
	Vector3f velocity(PX4_ISFINITE(_vehicle_local_position.vx) ? _vehicle_local_position.vx : 0.f,
			  PX4_ISFINITE(_vehicle_local_position.vy) ? _vehicle_local_position.vy : 0.f,
			  PX4_ISFINITE(_vehicle_local_position.vz) ? _vehicle_local_position.vz : 0.f);

	const bool setpoint_fresh = _vehicle_local_position_setpoint.timestamp != 0 &&
				    hrt_elapsed_time(&_vehicle_local_position_setpoint.timestamp) <=
				    TRACK_HANDOFF_SETPOINT_MAX_AGE;

	if (setpoint_fresh) {
		if (PX4_ISFINITE(_vehicle_local_position_setpoint.vx) &&
		    PX4_ISFINITE(_vehicle_local_position_setpoint.vy)) {
			velocity(0) = _vehicle_local_position_setpoint.vx;
			velocity(1) = _vehicle_local_position_setpoint.vy;
		}

		if (PX4_ISFINITE(_vehicle_local_position_setpoint.vz)) {
			velocity(2) = _vehicle_local_position_setpoint.vz;
		}
	}

	Vector2f velocity_xy(velocity(0), velocity(1));
	const float max_vel = math::max(_param_max_vel.get(), 0.1f);

	if (velocity_xy.norm() > max_vel) {
		velocity_xy = velocity_xy.normalized() * max_vel;
		velocity(0) = velocity_xy(0);
		velocity(1) = velocity_xy(1);
	}

	const float max_dz = math::max(_param_max_dz.get(), 0.1f);
	velocity(2) = math::constrain(velocity(2), -max_dz, max_dz);

	_track_handoff_velocity = velocity;
	_track_handoff_time = hrt_absolute_time();
	_track_handoff_velocity_valid = true;
	_velocity_sp = velocity;
	_acceleration_sp.zero();
	_last_track_setpoint_time = _track_handoff_time;
}

float DytGuidance::track_handoff_blend(hrt_abstime now) const
{
	if (!_track_handoff_velocity_valid || _track_handoff_time == 0) {
		return 1.f;
	}

	const float blend_time_s = math::max(_param_handoff_blend_time.get(), 0.f);

	if (blend_time_s <= 0.f) {
		return 1.f;
	}

	const float elapsed_s = (now - _track_handoff_time) * 1e-6f;
	float blend = math::constrain(elapsed_s / blend_time_s, 0.f, 1.f);
	blend = blend * blend * (3.f - 2.f * blend);
	return blend;
}

void DytGuidance::enter_state(TaskState new_state, uint8_t lost_reason)
{
	_state = new_state;
	_lost_reason = lost_reason;
	_state_enter_time = hrt_absolute_time();

	if (new_state == TaskState::SearchWaitLock) {
		_lock_streak = 0;
		_relock_streak = 0;
		_last_hint_lock_time = 0;
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		_midcourse_handoff_latched = _midcourse_handoff_latched || (_last_midcourse_target_time > 0);
		_next_midcourse_point_time = 0;
		_last_midcourse_point_time = 0;
		_midcourse_burst_remaining = 0;
		_midcourse_yaw_deg = NAN;
		_midcourse_pitch_deg = NAN;
		_track_handoff_time = 0;
		_track_handoff_velocity_valid = false;
		_net_release_sent = false;
		clear_net_release_trigger();
		clear_net_hold();
		clear_net_decel();
	} else if (new_state == TaskState::TrackFollow || new_state == TaskState::TrackIntercept) {
		send_dyt_geo_track_exit();
		_next_scan_time = 0;
		_search_pause_until = 0;
		_next_midcourse_point_time = 0;
		_last_midcourse_point_time = 0;
		_midcourse_burst_remaining = 0;
		_lost_streak = 0;
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
		capture_track_handoff_velocity();
	} else if (new_state == TaskState::LostHold) {
		_relock_streak = 0;
		_last_home_command_time = 0;
		_last_retrigger_time = 0;
		_last_hint_lock_time = 0;
		_next_midcourse_point_time = 0;
		_last_midcourse_point_time = 0;
		_midcourse_burst_remaining = 0;
		_midcourse_yaw_deg = NAN;
		_midcourse_pitch_deg = NAN;
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
		_track_handoff_time = 0;
		_track_handoff_velocity_valid = false;
		clear_net_release_trigger();
		clear_net_hold();
		clear_net_decel();
		clear_observations();
		capture_hold_setpoint();

		if (lost_reason != dyt_guidance_status_s::LOST_REASON_TIMEOUT &&
		    !update_midcourse_pointing(_state_enter_time, true)) {
			send_home_angle_command();
		}

		reset_search_scan(_state_enter_time);
	} else if (new_state == TaskState::Idle) {
		send_dyt_geo_track_exit();
		_midcourse_handoff_latched = false;
		clear_observations();
		_velocity_sp.zero();
		_acceleration_sp.zero();
		_yaw_sp = NAN;
		_yaw_rate_sp = NAN;
		_last_home_command_time = 0;
		_next_scan_time = 0;
		_next_midcourse_point_time = 0;
		_last_midcourse_point_time = 0;
		_midcourse_burst_remaining = 0;
		_midcourse_yaw_deg = NAN;
		_midcourse_pitch_deg = NAN;
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
		_track_handoff_time = 0;
		_track_handoff_velocity_valid = false;
		_net_release_sent = false;
		clear_net_release_trigger();
		clear_net_hold();
		clear_net_decel();
	} else if (new_state == TaskState::Abort) {
		send_dyt_geo_track_exit();
		_next_midcourse_point_time = 0;
		_last_midcourse_point_time = 0;
		_midcourse_burst_remaining = 0;
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		_candidate_ignore_until = 0;
		_candidate_ignored_sample_time = 0;
		_track_handoff_time = 0;
		_track_handoff_velocity_valid = false;
		_net_release_sent = false;
		clear_net_release_trigger();
		clear_net_hold();
		clear_net_decel();
	}
}

bool DytGuidance::activate_guidance(hrt_abstime now)
{
	_payload_lock_seen = false;
	_payload_lost_hold = false;
	_payload_lost_enter_time = 0;
	_last_home_command_time = 0;
	_last_retrigger_time = 0;
	_last_hint_lock_time = 0;
	_search_pause_until = 0;
	_next_scan_time = 0;

	if (!preconditions_ok()) {
		PX4_WARN("DYT guidance preconditions not met");
		return false;
	}

	clear_observations();
	capture_hold_setpoint();
	_requested_submode = dyt_guidance_status_s::SUBMODE_FOLLOW;
	enter_state(TaskState::SearchWaitLock);
	return true;
}

void DytGuidance::activate_guidance_and_request_lock(hrt_abstime now)
{
	if (activate_guidance(now)) {
		// The CGTD070 reports the tracking bbox only after tracking starts. A terminal-guidance
		// switch edge is the operator authorization to request green-box tracking directly.
		send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
		_last_hint_lock_time = now;
		_last_retrigger_time = now;
	}
}

void DytGuidance::deactivate_guidance(uint8_t lost_reason)
{
	_payload_lock_seen = false;
	_payload_lost_hold = false;
	_payload_lost_enter_time = 0;
	_last_home_command_time = 0;
	_last_retrigger_time = 0;
	_last_hint_lock_time = 0;
	_search_pause_until = 0;
	_next_scan_time = 0;
	send_dyt_geo_track_exit();
	send_dyt_command(dyt_command_s::CMD_STOP_TRACK);

	if (_state != TaskState::Idle) {
		enter_state(TaskState::Abort, lost_reason);
	}
}

void DytGuidance::abort_guidance(uint8_t lost_reason)
{
	send_dyt_geo_track_exit();
	send_dyt_command(dyt_command_s::CMD_STOP_TRACK);
	enter_state(TaskState::Abort, lost_reason);
}

void DytGuidance::enter_lost_hold(uint8_t lost_reason)
{
	enter_state(TaskState::LostHold, lost_reason);
}

bool DytGuidance::update_lost_reacquire(hrt_abstime now, hrt_abstime lost_enter_time)
{
	constexpr hrt_abstime HOME_COMMAND_INTERVAL{200_ms};
	const bool home_command_enabled = _state != TaskState::LostHold
					  || _lost_reason != dyt_guidance_status_s::LOST_REASON_TIMEOUT;
	const hrt_abstime enter_time = lost_enter_time > 0 ? lost_enter_time : _state_enter_time;
	const int32_t center_ms = math::max(_param_center_ms.get(), int32_t{0});
	const hrt_abstime center_delay = static_cast<hrt_abstime>(center_ms) * 1000ULL;

	if ((now - enter_time) < center_delay) {
		if (home_command_enabled && update_midcourse_pointing(now)) {
			_last_home_command_time = now;

		} else if (home_command_enabled &&
		    (_last_home_command_time == 0 || (now - _last_home_command_time) >= HOME_COMMAND_INTERVAL)) {
			send_home_angle_command();
		}

		return true;
	}

	return false;
}

void DytGuidance::update_payload_only_reacquire(hrt_abstime now)
{
	if (target_usable()) {
		_payload_lock_seen = true;
		_payload_lost_hold = false;
		_payload_lost_enter_time = 0;
		_last_retrigger_time = 0;
		_next_scan_time = 0;
		_search_pause_until = 0;
		return;
	}

	if (!_payload_lock_seen) {
		return;
	}

	if (!_payload_lost_hold) {
		_payload_lost_hold = true;
		_payload_lost_enter_time = now;
		_last_home_command_time = 0;
		_last_retrigger_time = 0;
		_last_hint_lock_time = 0;
		_search_pause_until = 0;
		clear_observations();
		if (!update_midcourse_pointing(now, true)) {
			send_home_angle_command();
		}
		reset_search_scan(now);
	}

	if (!update_lost_reacquire(now, _payload_lost_enter_time)) {
		// Search scan is disabled: point from shared target position until a target is visible.
		if (target_lock_candidate()) {
			update_hint_autolock(now);
		} else {
			update_midcourse_pointing(now);
		}
	}
}

void DytGuidance::send_dyt_command(uint8_t command, int16_t param_x, int16_t param_y, uint8_t param3, int8_t zoom_rate)
{
	dyt_command_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.command = command;
	msg.param_x = param_x;
	msg.param_y = param_y;
	msg.param3 = param3;
	msg.zoom_rate = zoom_rate;
	_dyt_command_pub.publish(msg);
	_last_command = command;
	_last_command_time = msg.timestamp;
	++_command_pub_count;
}

void DytGuidance::send_dyt_ownship_state(hrt_abstime now)
{
	if (!global_position_valid() || !PX4_ISFINITE(_vehicle_local_position.z)) {
		return;
	}

	if (_last_midcourse_ownship_time != 0 && (now - _last_midcourse_ownship_time) < MIDCOURSE_OWNSHIP_INTERVAL) {
		return;
	}

	const Eulerf euler(Quatf(_vehicle_attitude.q));
	const float ground_speed = (_vehicle_local_position.v_xy_valid && _vehicle_local_position.v_z_valid) ?
				   sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
					 _vehicle_local_position.vy * _vehicle_local_position.vy +
					 _vehicle_local_position.vz * _vehicle_local_position.vz) :
				   0.f;
	const bool airspeed_fresh = _airspeed_validated.timestamp != 0 && (now - _airspeed_validated.timestamp) < 1_s;
	const float airspeed = airspeed_fresh && PX4_ISFINITE(_airspeed_validated.true_airspeed_m_s) ?
			       _airspeed_validated.true_airspeed_m_s : ground_speed;

	dyt_command_s msg{};
	msg.timestamp = now;
	msg.command = dyt_command_s::CMD_SEND_OWNSHIP_STATE;
	msg.lat = _vehicle_global_position.lat;
	msg.lon = _vehicle_global_position.lon;
	msg.alt = _vehicle_global_position.alt;
	msg.rel_alt = -_vehicle_local_position.z;
	msg.roll_rad = euler.phi();
	msg.pitch_rad = euler.theta();
	msg.yaw_rad = euler.psi();
	msg.airspeed_m_s = airspeed;
	msg.groundspeed_m_s = ground_speed;
	_dyt_command_pub.publish(msg);
	_last_command = msg.command;
	_last_command_time = now;
	_last_midcourse_ownship_time = now;
	++_command_pub_count;
}

void DytGuidance::send_dyt_geo_track_target(hrt_abstime now, bool force)
{
	if (!midcourse_target_geo_valid()) {
		return;
	}

	if (!force && _last_midcourse_geo_target_time != 0 &&
	    (now - _last_midcourse_geo_target_time) < MIDCOURSE_GEO_TARGET_INTERVAL) {
		return;
	}

	dyt_command_s msg{};
	msg.timestamp = now;
	msg.command = dyt_command_s::CMD_GEO_TRACK;
	msg.lat = _midcourse_target_info.lat;
	msg.lon = _midcourse_target_info.lon;
	const float target_command_alt = midcourse_target_command_alt();

	if (!PX4_ISFINITE(target_command_alt)) {
		return;
	}

	msg.alt = target_command_alt;
	msg.rel_alt = midcourse_target_relative_alt();
	_dyt_command_pub.publish(msg);
	_last_command = msg.command;
	_last_command_time = now;
	_last_midcourse_geo_target_time = now;
	_midcourse_geotrack_active = true;
	++_command_pub_count;
}

void DytGuidance::send_dyt_geo_track_exit()
{
	if (!_midcourse_geotrack_active) {
		return;
	}

	send_dyt_command(dyt_command_s::CMD_GEO_TRACK_EXIT);
	_midcourse_geotrack_active = false;
	_last_midcourse_geo_target_time = 0;
	_last_midcourse_ownship_time = 0;
}

void DytGuidance::send_home_angle_command()
{
	const int16_t yaw_cmd = angle_deg_to_cdeg(_param_home_yaw_deg.get());
	const int16_t pitch_cmd = angle_deg_to_cdeg(_param_home_pitch_deg.get());
	send_dyt_command(dyt_command_s::CMD_CENTER_GIMBAL, yaw_cmd, pitch_cmd);
	_last_home_command_time = hrt_absolute_time();
}

int16_t DytGuidance::angle_deg_to_cdeg(float angle_deg) const
{
	if (!PX4_ISFINITE(angle_deg)) {
		angle_deg = 0.f;
	}

	const float limited_deg = math::constrain(angle_deg, -180.f, 180.f);
	return static_cast<int16_t>(roundf(limited_deg * 100.f));
}

bool DytGuidance::global_position_valid() const
{
	return _vehicle_global_position.lat_lon_valid && _vehicle_global_position.alt_valid &&
	       PX4_ISFINITE(static_cast<float>(_vehicle_global_position.lat)) &&
	       PX4_ISFINITE(static_cast<float>(_vehicle_global_position.lon)) &&
	       PX4_ISFINITE(_vehicle_global_position.alt);
}

bool DytGuidance::local_position_global_valid() const
{
	return _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid &&
	       _vehicle_local_position.xy_global && _vehicle_local_position.z_global &&
	       PX4_ISFINITE(_vehicle_local_position.x) && PX4_ISFINITE(_vehicle_local_position.y) &&
	       PX4_ISFINITE(_vehicle_local_position.z) &&
	       PX4_ISFINITE(_vehicle_local_position.ref_lat) &&
	       PX4_ISFINITE(_vehicle_local_position.ref_lon) &&
	       PX4_ISFINITE(_vehicle_local_position.ref_alt);
}

bool DytGuidance::midcourse_target_geo_valid() const
{
	if (_last_midcourse_target_time == 0) {
		return false;
	}

	const float timeout_s = math::constrain(_param_midcourse_target_timeout.get(), 0.1f, 30.f);

	return (hrt_absolute_time() - _last_midcourse_target_time) <= static_cast<hrt_abstime>(timeout_s * 1_s) &&
	       PX4_ISFINITE(static_cast<float>(_midcourse_target_info.lat)) &&
	       PX4_ISFINITE(static_cast<float>(_midcourse_target_info.lon)) &&
	       PX4_ISFINITE(static_cast<float>(_midcourse_target_info.alt));
}

float DytGuidance::midcourse_target_command_alt() const
{
	const float offset = _param_midcourse_target_alt_offset.get();
	const float finite_offset = PX4_ISFINITE(offset) ? offset : 0.f;
	const int32_t alt_mode = math::constrain(_param_midcourse_alt_mode.get(), int32_t{0}, int32_t{2});

	if (alt_mode == 0) {
		return static_cast<float>(_midcourse_target_info.alt) + finite_offset;
	}

	if (!global_position_valid()) {
		return NAN;
	}

	const float target_rel_alt = midcourse_target_relative_alt();
	const float own_rel_alt = own_midcourse_relative_alt();

	if (!PX4_ISFINITE(target_rel_alt) || !PX4_ISFINITE(own_rel_alt)) {
		return NAN;
	}

	return _vehicle_global_position.alt + (target_rel_alt - own_rel_alt) + finite_offset;
}

float DytGuidance::midcourse_target_relative_alt() const
{
	const float target_alt = static_cast<float>(_midcourse_target_info.alt);
	const int32_t alt_mode = math::constrain(_param_midcourse_alt_mode.get(), int32_t{0}, int32_t{2});

	if (!PX4_ISFINITE(target_alt)) {
		return NAN;
	}

	if (alt_mode == 2) {
		return target_alt;
	}

	if (alt_mode == 1) {
		// New links should send relative altitude. For older logs/firmware where
		// follower_info.alt is still AMSL, estimate the target's local reference
		// from the lowest observed target altitude in this boot.
		if (fabsf(target_alt) < 500.f) {
			return target_alt;
		}

		if (PX4_ISFINITE(_midcourse_target_alt_ref_m)) {
			return target_alt - _midcourse_target_alt_ref_m;
		}
	}

	return NAN;
}

float DytGuidance::own_midcourse_relative_alt() const
{
	if (_vehicle_local_position.z_valid && PX4_ISFINITE(_vehicle_local_position.z)) {
		return -_vehicle_local_position.z;
	}

	if (global_position_valid() && PX4_ISFINITE(_vehicle_local_position.ref_alt)) {
		return _vehicle_global_position.alt - _vehicle_local_position.ref_alt;
	}

	return NAN;
}

bool DytGuidance::midcourse_target_position_local(Vector3f &target_position) const
{
	if (!local_position_global_valid() || _last_midcourse_target_time == 0) {
		return false;
	}

	const float timeout_s = math::constrain(_param_midcourse_target_timeout.get(), 0.1f, 30.f);

	if ((hrt_absolute_time() - _last_midcourse_target_time) > static_cast<hrt_abstime>(timeout_s * 1_s)) {
		return false;
	}

	MapProjection map_ref{};
	map_ref.initReference(_vehicle_local_position.ref_lat, _vehicle_local_position.ref_lon,
			      _vehicle_local_position.ref_timestamp);

	float x = NAN;
	float y = NAN;
	map_ref.project(_midcourse_target_info.lat, _midcourse_target_info.lon, x, y);

	const float target_alt = midcourse_target_command_alt();

	if (!PX4_ISFINITE(x) || !PX4_ISFINITE(y) || !PX4_ISFINITE(target_alt)) {
		return false;
	}

	target_position(0) = x;
	target_position(1) = y;
	target_position(2) = static_cast<float>(_vehicle_local_position.ref_alt) - target_alt;

	return PX4_ISFINITE(target_position(2));
}

bool DytGuidance::compute_midcourse_gimbal_angle(float &yaw_deg, float &pitch_deg) const
{
	Vector3f target_position{};

	if (!midcourse_target_position_local(target_position)) {
		return false;
	}

	const Vector3f own_position(_vehicle_local_position.x, _vehicle_local_position.y, _vehicle_local_position.z);
	Vector3f los_ned = target_position - own_position;
	const float prediction_s = math::constrain(_param_midcourse_gimbal_prediction.get(), 0.f, 0.5f);

	if (prediction_s > 0.f) {
		const Vector3f target_velocity(static_cast<float>(_midcourse_target_info.vx),
					       static_cast<float>(_midcourse_target_info.vy),
					       static_cast<float>(_midcourse_target_info.vz));
		const Vector3f own_velocity(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
		const bool target_velocity_valid = PX4_ISFINITE(target_velocity(0)) && PX4_ISFINITE(target_velocity(1)) &&
						   PX4_ISFINITE(target_velocity(2));
		const bool own_velocity_valid = PX4_ISFINITE(own_velocity(0)) && PX4_ISFINITE(own_velocity(1)) &&
						PX4_ISFINITE(own_velocity(2));

		if (target_velocity_valid && own_velocity_valid) {
			const float target_age_s = _last_midcourse_target_time != 0 ?
						   math::constrain((hrt_absolute_time() - _last_midcourse_target_time) * 1e-6f, 0.f, 1.f) :
						   0.f;

			los_ned += target_velocity * (target_age_s + prediction_s) - own_velocity * prediction_s;
		}
	}

	if (!PX4_ISFINITE(los_ned(0)) || !PX4_ISFINITE(los_ned(1)) || !PX4_ISFINITE(los_ned(2))
	    || los_ned.norm_squared() < 1e-4f) {
		return false;
	}

	const int32_t gimbal_mode = math::constrain(_param_midcourse_gimbal_mode.get(), int32_t{0}, int32_t{1});
	float raw_yaw_deg = NAN;
	float raw_pitch_deg = NAN;
	const Dcmf body_to_ned(Quatf(_vehicle_attitude.q));
	const Vector3f los_body = body_to_ned.transpose() * los_ned;

	if (!PX4_ISFINITE(los_body(0)) || !PX4_ISFINITE(los_body(1)) || !PX4_ISFINITE(los_body(2))
	    || los_body.norm_squared() < 1e-4f) {
		return false;
	}

	// This payload has a body-vertical yaw joint followed by a pitch joint. Its
	// camera points up at pitch zero and forward at pitch -90 deg. Applying the
	// +90 deg installation pitch as a full Euler rotation puts the yaw axis on
	// its side, coupling yaw and pitch and amplifying azimuth near forward view.
	const float horizontal_norm = sqrtf(los_body(0) * los_body(0) + los_body(1) * los_body(1));
	const float body_yaw = atan2f(los_body(1), los_body(0));
	const float body_elevation = atan2f(-los_body(2), horizontal_norm);
	const float mount_yaw_deg = _param_mount_enable.get() > 0 ? finite_param_deg(_param_mount_yaw_deg.get()) : 0.f;
	const float mount_pitch_deg = _param_mount_enable.get() > 0 ? finite_param_deg(_param_mount_pitch_deg.get()) : 0.f;

	raw_yaw_deg = math::degrees(matrix::wrap_pi(body_yaw - math::radians(mount_yaw_deg)));

	if (gimbal_mode == 1) {
		float inertial_yaw_deg = NAN;

		if (!compute_midcourse_inertial_angles(los_ned, inertial_yaw_deg, raw_pitch_deg)) {
			return false;
		}

	} else {
		raw_pitch_deg = math::degrees(body_elevation) - mount_pitch_deg;
	}

	const float yaw_sign = static_cast<float>(_param_yaw_sign.get()) >= 0.f ? 1.f : -1.f;
	const float pitch_sign = static_cast<float>(_param_pitch_sign.get()) >= 0.f ? 1.f : -1.f;

	yaw_deg = (raw_yaw_deg - _param_yaw_off_deg.get()) / yaw_sign;
	pitch_deg = (raw_pitch_deg - _param_pitch_off_deg.get()) / pitch_sign;

	return PX4_ISFINITE(yaw_deg) && PX4_ISFINITE(pitch_deg);
}

bool DytGuidance::compute_midcourse_inertial_angles(const Vector3f &los_ned, float &yaw_deg, float &pitch_deg) const
{
	const float horizontal_norm = sqrtf(los_ned(0) * los_ned(0) + los_ned(1) * los_ned(1));

	if (!PX4_ISFINITE(horizontal_norm) || horizontal_norm < 1e-3f ||
	    !PX4_ISFINITE(los_ned(0)) || !PX4_ISFINITE(los_ned(1)) || !PX4_ISFINITE(los_ned(2))) {
		return false;
	}

	yaw_deg = math::degrees(atan2f(los_ned(1), los_ned(0)));
	pitch_deg = math::degrees(atan2f(-los_ned(2), horizontal_norm));

	return PX4_ISFINITE(yaw_deg) && PX4_ISFINITE(pitch_deg);
}

Vector3f DytGuidance::rotate_mount_los_to_body(const Vector3f &los_mount) const
{
	if (_param_mount_enable.get() <= 0) {
		return los_mount;
	}

	const Dcmf mount_to_body(Eulerf(math::radians(finite_param_deg(_param_mount_roll_deg.get())),
					math::radians(finite_param_deg(_param_mount_pitch_deg.get())),
					math::radians(finite_param_deg(_param_mount_yaw_deg.get()))));

	return mount_to_body * los_mount;
}

float DytGuidance::finite_param_deg(float value) const
{
	return PX4_ISFINITE(value) ? value : 0.f;
}

bool DytGuidance::update_midcourse_pointing(hrt_abstime now, bool force)
{
	if (_param_midcourse_geo_enable.get() > 0) {
		return update_midcourse_geo_tracking(now, force);
	}

	send_dyt_geo_track_exit();
	return update_midcourse_gimbal_pointing(now, force);
}

bool DytGuidance::update_midcourse_geo_tracking(hrt_abstime now, bool force)
{
	if (target_locked()) {
		// AUTO_LOCK already changed the payload out of geographic-follow mode. Clear
		// local bookkeeping without sending an exit command that would cancel tracking.
		_midcourse_geotrack_active = false;
		_last_midcourse_geo_target_time = 0;
		_last_midcourse_ownship_time = 0;
		return false;
	}

	if (!midcourse_target_geo_valid()) {
		send_dyt_geo_track_exit();
		return false;
	}

	send_dyt_ownship_state(now);
	send_dyt_geo_track_target(now, force);
	return true;
}

bool DytGuidance::update_midcourse_gimbal_pointing(hrt_abstime now, bool force)
{
	if (target_locked()) {
		return false;
	}

	float yaw_deg = NAN;
	float pitch_deg = NAN;

	if (!compute_midcourse_gimbal_angle(yaw_deg, pitch_deg)) {
		return false;
	}

	yaw_deg = math::constrain(yaw_deg, FRAME_YAW_MIN_DEG, FRAME_YAW_MAX_DEG);
	pitch_deg = math::constrain(pitch_deg, FRAME_PITCH_MIN_DEG, FRAME_PITCH_MAX_DEG);
	const bool have_previous_angle = PX4_ISFINITE(_midcourse_yaw_deg) && PX4_ISFINITE(_midcourse_pitch_deg);
	const float angle_delta_deg = have_previous_angle ?
				      math::max(fabsf(yaw_deg - _midcourse_yaw_deg), fabsf(pitch_deg - _midcourse_pitch_deg)) :
				      MIDCOURSE_RESEND_ANGLE_DELTA_DEG;
	const bool angle_changed = !have_previous_angle || angle_delta_deg >= MIDCOURSE_RESEND_ANGLE_DELTA_DEG;

	_midcourse_yaw_deg = yaw_deg;
	_midcourse_pitch_deg = pitch_deg;

	if (force || angle_changed) {
		_midcourse_burst_remaining = math::max(_midcourse_burst_remaining, MIDCOURSE_BURST_COUNT);

		if (_last_midcourse_point_time == 0 || (now - _last_midcourse_point_time) >= MIDCOURSE_BURST_INTERVAL) {
			_next_midcourse_point_time = now;

		} else {
			const hrt_abstime earliest_retry = _last_midcourse_point_time + MIDCOURSE_BURST_INTERVAL;

			if (_next_midcourse_point_time == 0 || _next_midcourse_point_time > earliest_retry) {
				_next_midcourse_point_time = earliest_retry;
			}
		}
	}

	if (!force && _next_midcourse_point_time != 0 && now < _next_midcourse_point_time) {
		return true;
	}

	if (!force && _last_midcourse_point_time != 0 && (now - _last_midcourse_point_time) < MIDCOURSE_BURST_INTERVAL) {
		return true;
	}

	send_angle_command(yaw_deg, pitch_deg, dyt_command_s::CMD_SET_FRAME_ANGLE);
	_last_midcourse_point_time = now;

	if (_midcourse_burst_remaining > 0) {
		--_midcourse_burst_remaining;
	}

	_next_midcourse_point_time = now + (_midcourse_burst_remaining > 0 ? MIDCOURSE_BURST_INTERVAL :
					    MIDCOURSE_HOLD_INTERVAL);
	return true;
}

void DytGuidance::retry_autolock(hrt_abstime now)
{
	const int32_t retry_ms = math::max(_param_retry_ms.get(), int32_t{100});
	const hrt_abstime retry_interval = static_cast<hrt_abstime>(retry_ms) * 1000ULL;

	if (_last_retrigger_time == 0 || (now - _last_retrigger_time) >= retry_interval) {
		send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
		_last_retrigger_time = now;
	}
}

bool DytGuidance::update_hint_autolock(hrt_abstime now)
{
	if (!target_lock_candidate()) {
		return false;
	}

	// 已经 LOCKED 时，不重复发锁定命令，但告诉外层：目标存在，应该停止搜索
	if (target_locked()) {
		return true;
	}

	const int32_t retry_ms = math::max(_param_retry_ms.get(), int32_t{100});
	const hrt_abstime retry_interval = static_cast<hrt_abstime>(retry_ms) * 1000ULL;

	if (_last_hint_lock_time == 0 || (now - _last_hint_lock_time) >= retry_interval) {
		send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
		_last_hint_lock_time = now;
		_last_retrigger_time = now;
		return true;
	}

	// 虽然没到重发时间，但目标候选仍然存在，所以外层必须停止搜索
	return true;
}

void DytGuidance::update_auto_activation(hrt_abstime now)
{
	if (_param_auto_enable.get() <= 0 || _state != TaskState::Idle || !preconditions_ok()) {
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		return;
	}

	if (!target_lock_candidate()) {
		_auto_lock_streak = 0;
		_auto_lock_last_sample_time = 0;
		return;
	}

	if (_last_target.timestamp_sample == _auto_lock_last_sample_time) {
		return;
	}

	_auto_lock_last_sample_time = _last_target.timestamp_sample;
	++_auto_lock_streak;

	const int32_t required_frames = math::constrain(_param_auto_frames.get(), int32_t{1}, int32_t{30});

	if (_auto_lock_streak < required_frames) {
		return;
	}

	if (!target_locked()) {
		send_dyt_command(dyt_command_s::CMD_AUTO_LOCK, -100);
	}

	activate_guidance(now);

	if (_state == TaskState::SearchWaitLock) {
		_last_hint_lock_time = now;
		_last_retrigger_time = now;
	}

	_auto_lock_streak = 0;
	_auto_lock_last_sample_time = 0;
}

bool DytGuidance::handle_lock_candidate_or_timeout(hrt_abstime now)
{
	constexpr hrt_abstime LOCK_CANDIDATE_MAX_HOLD = 1200_ms;
	constexpr hrt_abstime SCAN_UPDATE_DELAY = 100_ms;
	constexpr hrt_abstime CANDIDATE_COOLDOWN = 800_ms;

	if (!target_lock_candidate()) {
		_candidate_lock_active = false;
		_candidate_lock_start_time = 0;
		return false;
	}

	// 候选刚超时过，在冷却期内不要再拦截搜索
	// 除非来了新的 target sample
	if (_candidate_ignore_until > now &&
	    _last_target.timestamp_sample == _candidate_ignored_sample_time) {
		return false;
	}

	if (!_candidate_lock_active) {
		_candidate_lock_active = true;
		_candidate_lock_start_time = now;
	}

	const hrt_abstime held_time = now - _candidate_lock_start_time;

	if (held_time < LOCK_CANDIDATE_MAX_HOLD) {
		update_hint_autolock(now);
		_search_pause_until = now + LOCK_CANDIDATE_MAX_HOLD;
		_next_scan_time = now + SCAN_UPDATE_DELAY;
		_scan_segment_target_deg = NAN;
		return true;
	}

	// 候选等待超时：允许搜索继续，不要立刻重新拦截
	_candidate_lock_active = false;
	_candidate_lock_start_time = 0;
	_candidate_ignore_until = now + CANDIDATE_COOLDOWN;
	_candidate_ignored_sample_time = _last_target.timestamp_sample;
	_last_hint_lock_time = 0;
	_search_pause_until = 0;

	return false;
}

void DytGuidance::reset_search_scan(hrt_abstime now)
{
	_scan_region = ScanRegion::Center;
	_scan_center_passes = 0;
	_scan_row = 0;
	_scan_yaw_deg = 0.f;
	_scan_pitch_deg = 0.f;
	_scan_segment_target_deg = NAN;
	_search_pause_until = 0;

	const int32_t center_ms = math::max(_param_center_ms.get(), int32_t{0});
	_next_scan_time = now + static_cast<hrt_abstime>(center_ms) * 1000ULL;
}

int DytGuidance::scan_row_count(const ScanArea &area, float pitch_step_deg) const
{
	const float step_deg = (PX4_ISFINITE(pitch_step_deg) && pitch_step_deg > 0.f) ? pitch_step_deg : 10.f;
	const float pitch_span = math::max(area.pitch_top_deg - area.pitch_bottom_deg, 0.f);
	return static_cast<int>(floorf(pitch_span / step_deg)) + 1;
}

DytGuidance::ScanArea DytGuidance::active_scan_area() const
{
	if (_scan_region == ScanRegion::Center) {
		return {-60.f, 60.f, 30.f, -20.f};
	}

	return {-110.f, 110.f, 40.f, -100.f};
}

void DytGuidance::advance_search_scan_area(float pitch_step_deg)
{
	if (_scan_row < scan_row_count(active_scan_area(), pitch_step_deg)) {
		return;
	}

	_scan_row = 0;

	if (_scan_region == ScanRegion::Center) {
		++_scan_center_passes;

		if (_scan_center_passes >= SEARCH_CENTER_PASSES) {
			_scan_center_passes = 0;
			_scan_region = ScanRegion::Global;
		}

	} else {
		_scan_region = ScanRegion::Center;
		_scan_center_passes = 0;
	}
}

void DytGuidance::send_angle_command(float yaw_deg, float pitch_deg, uint8_t command)
{
	const float yaw_limited = math::constrain(yaw_deg, FRAME_YAW_MIN_DEG, FRAME_YAW_MAX_DEG);
	const float pitch_limited = math::constrain(pitch_deg, FRAME_PITCH_MIN_DEG, FRAME_PITCH_MAX_DEG);
	const int16_t yaw_cmd = static_cast<int16_t>(roundf(yaw_limited * 100.f));
	const int16_t pitch_cmd = static_cast<int16_t>(roundf(pitch_limited * 100.f));

	send_dyt_command(command, yaw_cmd, pitch_cmd);
}

void DytGuidance::update_search_scan(hrt_abstime now)
{
	constexpr float SCAN_UPDATE_INTERVAL_S = 0.10f;

	// 已经 LOCKED 且新鲜，才真正禁止搜索
	if (target_locked() && target_fresh()) {
		_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
		_search_pause_until = now + 300_ms;
		_scan_segment_target_deg = NAN;
		return;
	}

	// 只是候选目标：触发锁定，但不要永久挡住搜索
	if (target_lock_candidate()) {
		update_hint_autolock(now);

		if (_last_hint_lock_time != 0 && (now - _last_hint_lock_time) < 300_ms) {
			_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
			return;
		}

		// 注意：这里不要 return，让搜索逻辑继续执行
	}

	if (_next_scan_time == 0) {
		reset_search_scan(now);
	}

	if (now < _next_scan_time) {
		return;
	}

	if (_search_pause_until > now) {
		_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
		return;
	}

	const float pitch_step_deg = math::constrain(_param_scan_pitch_step.get(), 1.f, 30.f);
	advance_search_scan_area(pitch_step_deg);

	const ScanArea area = active_scan_area();
	const bool forward = (_scan_row % 2) == 0;
	const float row_end_deg = forward ? area.yaw_max_deg : area.yaw_min_deg;
	float pitch_deg = area.pitch_top_deg - static_cast<float>(_scan_row) * pitch_step_deg;

	if (pitch_deg < area.pitch_bottom_deg) {
		pitch_deg = area.pitch_bottom_deg;
	}

	const float yaw_speed_deg_s = math::constrain(_param_scan_yaw_speed.get(), 0.1f, 90.f);
	const float edge_pause_s = math::constrain(_param_scan_edge_pause.get(), 0.f, 2.f);
	const float yaw_step_deg = math::constrain(_param_scan_yaw_step.get(), 5.f, 60.f);
	const float dwell_s = math::constrain(_param_scan_dwell.get(), 0.1f, 2.f);

	float current_yaw_deg = _scan_yaw_deg;

	if (_have_target && target_fresh() && PX4_ISFINITE(_last_target.gimbal_yaw_rad)) {
		current_yaw_deg = math::degrees(_last_target.gimbal_yaw_rad);
	}

	if (!PX4_ISFINITE(_scan_segment_target_deg)) {
		if (fabsf(row_end_deg - current_yaw_deg) <= yaw_step_deg * 0.5f) {
			++_scan_row;
			advance_search_scan_area(pitch_step_deg);
			_scan_segment_target_deg = NAN;
			_search_pause_until = now + static_cast<hrt_abstime>(edge_pause_s * 1e6f);
			_scan_yaw_deg = current_yaw_deg;
			_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
			return;
		}

		if (forward) {
			_scan_segment_target_deg = math::min(current_yaw_deg + yaw_step_deg, row_end_deg);
		} else {
			_scan_segment_target_deg = math::max(current_yaw_deg - yaw_step_deg, row_end_deg);
		}
	}

	const float target_yaw = _scan_segment_target_deg;
	const float travel_deg = fabsf(target_yaw - current_yaw_deg);
	const float travel_s = (yaw_speed_deg_s > 0.1f) ? (travel_deg / yaw_speed_deg_s) : 1.f;

	send_angle_command(target_yaw, pitch_deg, dyt_command_s::CMD_SET_FRAME_ANGLE);

	const bool at_row_end = fabsf(target_yaw - row_end_deg) < 1.f;

	if (at_row_end) {
		_scan_segment_target_deg = NAN;
	} else {
		if (forward) {
			_scan_segment_target_deg = math::min(target_yaw + yaw_step_deg, row_end_deg);
		} else {
			_scan_segment_target_deg = math::max(target_yaw - yaw_step_deg, row_end_deg);
		}
	}

	const float wait_s = travel_s + dwell_s;
	_search_pause_until = now + static_cast<hrt_abstime>(wait_s * 1e6f);
	_scan_yaw_deg = target_yaw;
	_scan_pitch_deg = pitch_deg;
	_next_scan_time = now + static_cast<hrt_abstime>(SCAN_UPDATE_INTERVAL_S * 1e6f);
}

void DytGuidance::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	update_params_if_needed();
	update_vehicle_id();
	update_subscriptions();

	const hrt_abstime now = hrt_absolute_time();
	const bool activation_request = activation_requested();
	const bool midcourse_pointing_request = midcourse_pointing_requested();
	const bool auto_activation_enabled = _param_auto_enable.get() > 0;
	const bool intercept_request = aux_switch_active(_param_int_aux.get());
	const bool activation_rising = activation_request && !_prev_activation_request;

	if (activation_rising) {
		activate_guidance_and_request_lock(now);
	}

	if (auto_activation_enabled) {
		if (activation_request) {
			update_auto_activation(now);

		} else {
			_auto_lock_streak = 0;
			_auto_lock_last_sample_time = 0;

			if (_prev_activation_request) {
				deactivate_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
			}
		}

	} else {
		if (!activation_request && _prev_activation_request) {
			deactivate_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
		}
	}

	_prev_activation_request = activation_request;
	_requested_submode = intercept_request ? dyt_guidance_status_s::SUBMODE_INTERCEPT :
			     dyt_guidance_status_s::SUBMODE_FOLLOW;

	if (!auto_activation_enabled && activation_request && (_state == TaskState::Idle || _state == TaskState::Abort)) {
		update_payload_only_reacquire(now);
	}

	if (_state != TaskState::Idle && _state != TaskState::Abort) {
		if (!preconditions_ok()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_PRECONDITION);
		} else if ((now - _state_enter_time) > MANUAL_TAKEOVER_GRACE && manual_takeover_detected()) {
			abort_guidance(dyt_guidance_status_s::LOST_REASON_MANUAL);
		}
	}

	if (_net_hold_active && _state != TaskState::Idle && _state != TaskState::Abort) {
		request_offboard_mode();

		if (_net_brake_active) {
			publish_offboard_mode(false);
			publish_net_brake_setpoint();

		} else {
			publish_offboard_mode(true);
			publish_hold_setpoint();
		}

		publish_status();
		return;
	}

	switch (_state) {
	case TaskState::Idle:
		if (midcourse_pointing_request && !activation_request) {
			// Midcourse-only operation: keep the payload geographically pointed
			// while cooperative_rendezvous owns aircraft motion.
			update_midcourse_pointing(now);

		} else if (_midcourse_geotrack_active && !midcourse_pointing_request) {
			send_dyt_geo_track_exit();
		}

		break;

	case TaskState::SearchWaitLock:
		if (target_usable()) {
			++_lock_streak;

			if (_lock_streak >= _param_lock_frames.get()) {
				enter_state(TaskState::TrackFollow);
			}

		} else {
			_lock_streak = 0;

			if (target_lock_candidate()) {
				update_hint_autolock(now);
				_search_pause_until = now + 1200_ms;
				_next_scan_time = now + 100_ms;
			} else {
				update_midcourse_pointing(now);
			}

			if ((now - _state_enter_time) > static_cast<hrt_abstime>(_param_wait_ms.get()) * 1000ULL) {
				enter_lost_hold(dyt_guidance_status_s::LOST_REASON_TIMEOUT);
			}
		}
		break;

	case TaskState::TrackFollow:
		if (!target_usable()) {
			++_lost_streak;

			if (_lost_streak >= _param_relock_frames.get()) {
				enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
						 dyt_guidance_status_s::LOST_REASON_TRACKING);
			}

		} else {
			_lost_streak = 0;

			if (intercept_request && intercept_allowed()) {
				enter_state(TaskState::TrackIntercept);
			}
		}
		break;

	case TaskState::TrackIntercept:
		if (!target_usable()) {
			++_lost_streak;

			if (_lost_streak >= _param_relock_frames.get()) {
				enter_lost_hold(target_locked() ? dyt_guidance_status_s::LOST_REASON_STALE :
						 dyt_guidance_status_s::LOST_REASON_TRACKING);
			}

		} else {
			_lost_streak = 0;

			if (!intercept_request || !intercept_allowed()) {
				enter_state(TaskState::TrackFollow);
			}
		}
		break;

	case TaskState::LostHold:
		if (target_usable()) {
			++_relock_streak;

			if (_relock_streak >= _param_relock_frames.get()) {
				enter_state(intercept_request && intercept_allowed() ? TaskState::TrackIntercept : TaskState::TrackFollow);
			}

		} else {
			_relock_streak = 0;

			const int32_t center_ms = math::max(_param_center_ms.get(), int32_t{0});
			const hrt_abstime center_delay = static_cast<hrt_abstime>(center_ms) * 1000ULL;
			const hrt_abstime lost_timeout = static_cast<hrt_abstime>(math::max(_param_lost_ms.get(), int32_t{0})) * 1000ULL;
			const bool center_done = (now - _state_enter_time) >= center_delay;

			// 建议：DYTG_LOSTMS=0 时不要自动停止搜索
			if (center_done && lost_timeout > 0 && (now - _state_enter_time) > center_delay + lost_timeout) {
				abort_guidance(_lost_reason);
			} else if (center_done) {
				// Search scan is disabled: use shared target position to point the seeker, then lock when visible.
				if (target_lock_candidate()) {
					update_hint_autolock(now);
				} else {
					update_midcourse_pointing(now);
				}
			} else {
				update_midcourse_pointing(now);
			}
		}
		break;

	case TaskState::Abort:
		enter_state(TaskState::Idle, dyt_guidance_status_s::LOST_REASON_NONE);
		break;
	}

	if (_state == TaskState::SearchWaitLock || _state == TaskState::LostHold) {
		update_midcourse_pointing(now);

		// During midcourse handoff the seeker only drives the gimbal while searching / lost;
		// aircraft motion stays with the position-sharing follower so there is no hover
		// break before visual lock.
		if (!midcourse_handoff_active()) {
			request_offboard_mode();
			publish_offboard_mode(true);
			publish_hold_setpoint();
		}
	}

	if (_state == TaskState::TrackFollow) {
		request_offboard_mode();
		publish_offboard_mode(false);
		publish_track_setpoint(follow_profile());
	}

	if (_state == TaskState::TrackIntercept) {
		request_offboard_mode();
		publish_offboard_mode(false);
		publish_track_setpoint(intercept_profile());
	}

	publish_status();
}

int DytGuidance::task_spawn(int argc, char *argv[])
{
	DytGuidance *instance = new DytGuidance();

	if (instance != nullptr) {
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

int DytGuidance::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return print_usage("module not running");
	}

	if (!strcmp(argv[0], "status")) {
		get_instance()->show_status();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "activate")) {
		get_instance()->_manual_activation = true;
		get_instance()->_prev_activation_request = false;
		return PX4_OK;
	}

	if (!strcmp(argv[0], "deactivate")) {
		get_instance()->_manual_activation = false;
		return PX4_OK;
	}

	if (!strcmp(argv[0], "retrigger")) {
		get_instance()->send_dyt_command(dyt_command_s::CMD_RETRIGGER, -100);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int DytGuidance::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
DYT visual guidance controller that consumes DYT telemetry, maintains a small LOS observation buffer,
and publishes offboard trajectory setpoints for follow and intercept behaviors.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dyt_guidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("activate");
	PRINT_MODULE_USAGE_COMMAND("deactivate");
	PRINT_MODULE_USAGE_COMMAND("retrigger");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int dyt_guidance_main(int argc, char *argv[])
{
	return DytGuidance::main(argc, argv);
}
