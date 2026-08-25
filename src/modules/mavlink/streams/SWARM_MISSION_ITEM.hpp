/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef SWARM_MISSION_ITEM_HPP
#define SWARM_MISSION_ITEM_HPP

#include <uORB/topics/swarm_mission_item.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

class MavlinkStreamSwarmMissionItem : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSwarmMissionItem(mavlink); }

	static constexpr const char *get_name_static() { return "SWARM_MISSION_ITEM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SWARM_MISSION_ITEM; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _swarm_mission_item_sub.advertised() ?
		       MAVLINK_MSG_ID_SWARM_MISSION_ITEM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamSwarmMissionItem(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _swarm_mission_item_sub{ORB_ID(swarm_mission_item)};

	bool send() override
	{
		swarm_mission_item_s item;

		bool has_update = _swarm_mission_item_sub.update(&item);

		if (has_update) {
			mavlink_swarm_mission_item_t msg{};

			msg.timestamp = item.timestamp;
			msg.group_id = item.group_id;
			msg.leader_id = item.leader_id;
			msg.mission_id = item.mission_id;
			msg.total_count = item.total_count;
			msg.current_seq = item.current_seq;
			msg.seq = item.seq;
			msg.nav_cmd = item.nav_cmd;
			msg.lat = item.lat;
			msg.lon = item.lon;
			msg.alt = item.alt;
			msg.yaw = item.yaw;
			msg.acceptance_radius = item.acceptance_radius;
			msg.loiter_radius = item.loiter_radius;
			msg.time_inside = item.time_inside;
			msg.autocontinue = item.autocontinue ? 1 : 0;
			msg.sync_type = item.sync_type;

			mavlink_msg_swarm_mission_item_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // SWARM_MISSION_ITEM_HPP
