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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#ifndef UAV_ROLE_HPP
#define UAV_ROLE_HPP

#include <drivers/drv_hrt.h>
#include <uORB/topics/dyt_guidance_status.h>

class MavlinkStreamUavRole : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamUavRole(mavlink); }

	static constexpr const char *get_name_static() { return "UAV_ROLE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_UAV_ROLE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _status_sub.advertised() ? MAVLINK_MSG_ID_UAV_ROLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamUavRole(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _status_sub{ORB_ID(dyt_guidance_status)};

	bool send() override
	{
		dyt_guidance_status_s status{};

		if (!_status_sub.copy(&status) || status.timestamp == 0 || status.timestamp > hrt_absolute_time()
		    || hrt_elapsed_time(&status.timestamp) > 1_s) {
			return false;
		}

		mavlink_uav_role_t msg{};
		msg.mavid = _mavlink->get_system_id();
		msg.vehicle_type = status.vehicle_type;
		mavlink_msg_uav_role_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // UAV_ROLE_HPP
