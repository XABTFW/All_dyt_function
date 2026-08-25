/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef UAV_INFO_HPP
#define UAV_INFO_HPP

#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/cooperative_position.h>
#include <lib/geo/geo.h>
#include <drivers/drv_hrt.h>

class MavlinkStreamUavInfo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamUavInfo(mavlink); }

	static constexpr const char *get_name_static() { return "UAV_INFO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_UAV_INFO; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return (_lpos_sub.advertised() || _cooperative_position_sub.advertised()) ?
		       MAVLINK_MSG_ID_UAV_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamUavInfo(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		// 缓存参数句柄，避免每次send()都调用param_find()
		_param_leader = param_find("SWARM_SET_LEADER");
		_param_group = param_find("SWARM_GROUP_ID");
	}

	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _trajectory_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _cooperative_position_sub{ORB_ID(cooperative_position)};

	MapProjection _global_local_proj_ref{};
	double _ref_lat{0.0};
	double _ref_lon{0.0};

	// 用于判断是否为主机（每次都检查，支持动态切换）
	bool _is_leader{false};

	// 缓存参数句柄
	param_t _param_leader{PARAM_INVALID};
	param_t _param_group{PARAM_INVALID};

	bool send() override
	{
		cooperative_position_s cooperative_position{};

		if (_cooperative_position_sub.copy(&cooperative_position) &&
		    cooperative_position.timestamp != 0 &&
		    (hrt_absolute_time() - cooperative_position.timestamp) < 300000 &&
		    cooperative_position.mavid == static_cast<uint32_t>(_mavlink->get_system_id())) {
			mavlink_uav_info_t msg{};
			msg.mavid = cooperative_position.mavid;
			msg.group_id = 0;
			msg.is_leader = 0;
			msg.lat = cooperative_position.lat;
			msg.lon = cooperative_position.lon;
			msg.rel_alt = cooperative_position.alt;
			msg.vx = cooperative_position.vx;
			msg.vy = cooperative_position.vy;
			msg.vz = cooperative_position.vz;
			msg.yaw = cooperative_position.yaw;
			msg.yaw_speed = cooperative_position.yawspeed;
			msg.land = 0;

			mavlink_msg_uav_info_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		// 每次都检查是否为主机，支持 QGC 动态切换主机
		int32_t swarm_set_leader = 0;
		if (_param_leader != PARAM_INVALID) {
			param_get(_param_leader, &swarm_set_leader);
			_is_leader = (swarm_set_leader == 1);
		}

		// 获取组号
		int32_t group_id = 1;
		if (_param_group != PARAM_INVALID) {
			param_get(_param_group, &group_id);
		}

		// 始终发送位置信息，不依赖于位置是否更新
		// 这样即使主机静止悬停，从机也能持续收到主机位置

		vehicle_local_position_s local_pos{};
		vehicle_global_position_s global_pos{};
		trajectory_setpoint_s trajectory{};
		vehicle_status_s vehicle_status{};
		sensor_gps_s gps{};
		home_position_s home{};

		_lpos_sub.copy(&local_pos);
		_gpos_sub.copy(&global_pos);
		_trajectory_sub.copy(&trajectory);
		_vehicle_status_sub.copy(&vehicle_status);
		_gps_sub.copy(&gps);
		_home_position_sub.copy(&home);

		const bool global_alt_valid = global_pos.timestamp != 0 && global_pos.alt_valid && PX4_ISFINITE(global_pos.alt);

		// 填充 MAVLink 消息
		mavlink_uav_info_t msg{};;

		// 主机：发送目标设定点（trajectory），让从机跟随
		// 从机：发送真实位置（GPS + local_pos），用于避障
		if (_is_leader) {
			// 主机：需要将目标设定点转换为全局经纬度
			// 检查参考位置是否有效
			if (!PX4_ISFINITE(local_pos.ref_lat) || !PX4_ISFINITE(local_pos.ref_lon)) {
				return false;
			}

			// 检查参考位置是否变化，如果变化则重新初始化投影
			if (fabs(local_pos.ref_lat - _ref_lat) > 1e-7 || fabs(local_pos.ref_lon - _ref_lon) > 1e-7) {
				_global_local_proj_ref.initReference(local_pos.ref_lat, local_pos.ref_lon, hrt_absolute_time());
				_ref_lat = local_pos.ref_lat;
				_ref_lon = local_pos.ref_lon;
			}

			// 将目标设定点转换为全局经纬度
			double lat, lon;
			_global_local_proj_ref.reproject(trajectory.position[0], trajectory.position[1], lat, lon);

			msg.lat = lat;
			msg.lon = lon;
			// 将 NED 坐标系的 z（负值）转换为全局高度（MSL）
			// trajectory.position[2] 是 NED 的 z（向下为正，所以是负值）
			// local_pos.ref_alt 是参考点的 MSL 高度
			// 全局高度 = ref_alt - NED_z
			msg.rel_alt = local_pos.ref_alt - trajectory.position[2];  // 转换为 MSL 高度
			msg.vx = trajectory.velocity[0];       // 目标速度
			msg.vy = trajectory.velocity[1];
			msg.vz = trajectory.velocity[2];
		} else {
			if (!global_alt_valid) {
				return false;
			}

			// 从机：真实位置使用 GPS 经纬度 + 融合后的全局高度，和 DYT 本机高度源保持一致
			msg.lat = gps.latitude_deg;
			msg.lon = gps.longitude_deg;
			msg.rel_alt = global_pos.alt;      // vehicle_global_position 高度（AMSL）
			msg.vx = local_pos.vx;             // 真实速度
			msg.vy = local_pos.vy;
			msg.vz = local_pos.vz;
		}

		msg.yaw = local_pos.heading;
		msg.yaw_speed = local_pos.delta_heading;

		// ★★★ 主机在yaw字段中携带home.z，用于从机高度补偿 ★★★
		if (_is_leader && home.valid_lpos) {
			// 使用特殊编码：yaw = home.z + 1000
			// 这样从机可以识别并提取home.z值
			msg.yaw = home.z + 1000.0f;
		}

		// 检查是否处于降落状态（Bit 0）
		// 检查是否已到达目标位置（Bit 1）- 用于避撞优先级
		// land字段: Bit 0 = landing, Bit 1 = at_target
		uint32_t land_flags = 0;
		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
			land_flags |= 0x01;  // Bit 0: landing
		}

		// 从机：检查是否已到达目标位置（通过速度判断，速度很小说明已到位）
		// 主机始终设置为0（主机不需要避让）
		if (!_is_leader) {
			// 从机：速度小于0.5m/s认为已到达目标位置
			float speed_xy = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
			if (speed_xy < 0.5f) {
				land_flags |= 0x02;  // Bit 1: at_target
			}
		}
		msg.land = land_flags;

		// 获取系统 ID
		msg.mavid = _mavlink->get_system_id();
		msg.group_id = group_id;
		msg.is_leader = _is_leader ? 1 : 0;

		// 发送第一条消息（主机发送目标位置用于跟随，从机发送真实位置）
		mavlink_msg_uav_info_send_struct(_mavlink->get_channel(), &msg);

		// 主机额外发送一条真实位置消息，用于避撞
		// 这条消息的 mavid 加上偏移量（+100），接收端识别后用于避撞
		// 这样主机同时发送：目标位置（跟随用）+ 真实位置（避撞用）
		if (_is_leader && global_alt_valid) {
			mavlink_uav_info_t msg_real{};
			msg_real.lat = gps.latitude_deg;
			msg_real.lon = gps.longitude_deg;
			msg_real.rel_alt = global_pos.alt;
			msg_real.vx = local_pos.vx;
			msg_real.vy = local_pos.vy;
			msg_real.vz = local_pos.vz;
			msg_real.yaw = local_pos.heading;
			msg_real.yaw_speed = local_pos.delta_heading;
			msg_real.land = msg.land;
			msg_real.group_id = group_id;
			msg_real.is_leader = 1;
			// 使用特殊标记：mavid + 100，表示这是主机的真实位置（用于避撞）
			msg_real.mavid = _mavlink->get_system_id() + 100;

			mavlink_msg_uav_info_send_struct(_mavlink->get_channel(), &msg_real);
		}

		return true;
	}
};

#endif // UAV_INFO_HPP
