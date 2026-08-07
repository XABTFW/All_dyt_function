/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/cooperative_position.h>

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
		return _cooperative_position_sub.advertised() ?
		       MAVLINK_MSG_ID_UAV_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamUavInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _cooperative_position_sub{ORB_ID(cooperative_position)};

	bool send() override
	{
		cooperative_position_s position{};

		if (!_cooperative_position_sub.copy(&position) || position.timestamp == 0 ||
		    hrt_elapsed_time(&position.timestamp) > 300_ms ||
		    position.mavid != static_cast<uint32_t>(_mavlink->get_system_id())) {
			return false;
		}

		mavlink_uav_info_t msg{};
		msg.mavid = position.mavid;
		msg.lat = position.lat;
		msg.lon = position.lon;
		msg.rel_alt = position.alt;
		msg.vx = position.vx;
		msg.vy = position.vy;
		msg.vz = position.vz;
		msg.yaw = position.yaw;
		msg.yaw_speed = position.yawspeed;
		mavlink_msg_uav_info_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};
