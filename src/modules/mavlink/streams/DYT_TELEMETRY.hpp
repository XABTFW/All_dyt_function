/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <string.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/dyt_guidance_status.h>
#include <uORB/topics/dyt_status_reply.h>
#include <uORB/topics/dyt_target.h>
#include <uORB/topics/sdm50_status.h>

class MavlinkStreamDytTelemetry : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDytTelemetry(mavlink); }

	static constexpr const char *get_name_static() { return "DYT_TELEMETRY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DYT_SYSTEM_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		unsigned size{0};

		if (_guidance_status_sub.advertised()) {
			size += MAVLINK_MSG_ID_DYT_SYSTEM_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		if (_target_sub.advertised()) {
			size += MAVLINK_MSG_ID_DYT_TARGET_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		if (_reply_sub.advertised()) {
			size += MAVLINK_MSG_ID_DYT_STATUS_REPLY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		if (_sdm50_status_sub.advertised()) {
			size += MAVLINK_MSG_ID_SDM50_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return size;
	}

private:
	explicit MavlinkStreamDytTelemetry(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _guidance_status_sub{ORB_ID(dyt_guidance_status)};
	uORB::Subscription _target_sub{ORB_ID(dyt_target)};
	uORB::Subscription _reply_sub{ORB_ID(dyt_status_reply)};
	uORB::Subscription _sdm50_status_sub{ORB_ID(sdm50_status)};

	bool send() override
	{
		bool sent{false};
		dyt_guidance_status_s status{};

		if (_guidance_status_sub.update(&status)) {
			mavlink_dyt_system_status_t msg{};
			msg.time_boot_ms = status.timestamp / 1000ULL;
			msg.command_sequence = status.command_sequence;
			msg.net_trigger_count = status.net_trigger_count;
			msg.los_age_s = status.los_age_s;
			msg.frame_dt_s = status.frame_dt_s;
			msg.delay_s = status.delay_s;
			memcpy(msg.los_ned, status.los_ned, sizeof(msg.los_ned));
			memcpy(msg.omega_los_ned, status.omega_los_ned, sizeof(msg.omega_los_ned));
			memcpy(msg.velocity_sp, status.velocity_sp, sizeof(msg.velocity_sp));
			memcpy(msg.acceleration_sp, status.acceleration_sp, sizeof(msg.acceleration_sp));
			msg.yaw_sp = status.yaw_sp;
			msg.yaw_rate_sp = status.yaw_rate_sp;
			msg.status_flags = (status.guidance_phase != dyt_guidance_status_s::PHASE_DISARMED ? 1u : 0u)
					   | (status.active ? 1u << 1 : 0u)
					   | (status.target_locked ? 1u << 2 : 0u)
					   | (status.controlling_vehicle ? 1u << 3 : 0u)
					   | (status.target_fresh ? 1u << 4 : 0u)
					   | (status.intercept_allowed ? 1u << 5 : 0u)
					   | (status.midcourse_active ? 1u << 6 : 0u)
					   | (status.midcourse_target_valid ? 1u << 7 : 0u)
					   | (status.net_trigger_sent ? 1u << 8 : 0u);
			msg.vehicle_type = status.vehicle_type;
			msg.control_mode = status.control_mode;
			msg.semi_auto_state = status.semi_auto_state;
			msg.guidance_phase = status.guidance_phase;
			msg.gcs_phase_request = status.gcs_phase_request;
			msg.command_phase = status.command_phase;
			msg.command_result = status.command_result;
			msg.guidance_state = status.state;
			msg.requested_submode = status.requested_submode;
			msg.active_submode = status.active_submode;
			msg.lost_reason = status.lost_reason;
			mavlink_msg_dyt_system_status_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		dyt_target_s target{};

		if (_target_sub.update(&target)) {
			mavlink_dyt_target_status_t msg{};
			msg.time_boot_ms = target.timestamp / 1000ULL;
			msg.time_sample_ms = target.timestamp_sample / 1000ULL;
			msg.frame_counter = target.frame_counter;
			msg.los_x_rad = target.los_x_rad;
			msg.los_y_rad = target.los_y_rad;
			msg.gimbal_roll_rad = target.gimbal_roll_rad;
			msg.gimbal_pitch_frame_rad = target.gimbal_pitch_frame_rad;
			msg.gimbal_pitch_rad = target.gimbal_pitch_rad;
			msg.gimbal_yaw_rad = target.gimbal_yaw_rad;
			msg.gimbal_roll_rate_rad_s = target.gimbal_roll_rate_rad_s;
			msg.gimbal_pitch_rate_rad_s = target.gimbal_pitch_rate_rad_s;
			msg.gimbal_yaw_rate_rad_s = target.gimbal_yaw_rate_rad_s;
			msg.bbox_width_px = target.bbox_width_px;
			msg.bbox_height_px = target.bbox_height_px;
			msg.range_m = target.range_m;
			msg.zoom_ratio = target.zoom_ratio;
			msg.frame_dt_s = target.frame_dt_s;
			msg.last_rx_age_s = target.last_rx_age_s;
			msg.parse_error_count = target.parse_error_count;
			msg.target_flags = (target.target_valid ? 1u : 0u)
					 | (target.auto_hint ? 1u << 1 : 0u)
					 | (target.image_enhance ? 1u << 2 : 0u)
					 | (target.recording ? 1u << 3 : 0u)
					 | (target.motor_on ? 1u << 4 : 0u)
					 | (target.follow_mode ? 1u << 5 : 0u)
					 | (target.laser_on ? 1u << 6 : 0u)
					 | (target.selftest_done ? 1u << 7 : 0u)
					 | (target.gyro_calib_failed ? 1u << 8 : 0u)
					 | (target.servo_fault ? 1u << 9 : 0u)
					 | (target.image_board_fault ? 1u << 10 : 0u);
			msg.tracking_state = target.tracking_state;
			// Expose the payload's raw image-source status to the ground station:
			// 0x01 visible, 0x02 infrared. Keep target.video_source as the
			// internal four-source enum used by flight-side processing.
			msg.video_source = target.status3;
			msg.tracking_algorithm = target.tracking_algorithm;
			msg.status1 = target.status1;
			msg.status2 = target.status2;
			msg.status3 = target.status3;
			msg.self_test_raw = target.self_test_raw;
			mavlink_msg_dyt_target_status_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		dyt_status_reply_s reply{};

		if (_reply_sub.update(&reply)) {
			mavlink_dyt_status_reply_t msg{};
			msg.time_boot_ms = reply.timestamp / 1000ULL;
			msg.time_sample_ms = reply.timestamp_sample / 1000ULL;
			msg.parse_error_count = reply.parse_error_count;
			msg.control_code = reply.control_code;
			msg.param_length = reply.param_length;
			msg.truncated = reply.truncated;
			memcpy(msg.params, reply.params, sizeof(msg.params));
			mavlink_msg_dyt_status_reply_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		sdm50_status_s sdm50_status{};

		if (_sdm50_status_sub.update(&sdm50_status)) {
			mavlink_sdm50_status_t msg{};
			msg.time_boot_ms = sdm50_status.timestamp / 1000ULL;
			msg.time_sample_ms = sdm50_status.timestamp_sample / 1000ULL;
			msg.device_id = sdm50_status.device_id;
			msg.distance_m = sdm50_status.distance_m;
			msg.closing_speed_m_s = sdm50_status.closing_speed_m_s;
			msg.status = sdm50_status.valid ? 1 : 0;
			mavlink_msg_sdm50_status_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};
