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

#ifndef SWARM_OPERATION_ACK_HPP
#define SWARM_OPERATION_ACK_HPP

#include <uORB/topics/swarm_operation_ack.h>
#include <drivers/drv_hrt.h>

class MavlinkStreamSwarmOperationAck : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSwarmOperationAck(mavlink); }

	static constexpr const char *get_name_static() { return "SWARM_OPERATION_ACK"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SWARM_OPERATION_ACK; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _swarm_op_ack_sub.advertised() ? MAVLINK_MSG_ID_SWARM_OPERATION_ACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamSwarmOperationAck(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _swarm_op_ack_sub{ORB_ID(swarm_operation_ack)};

	bool send() override
	{
		swarm_operation_ack_s ack;

		if (_swarm_op_ack_sub.update(&ack)) {
			mavlink_swarm_operation_ack_t msg{};
			msg.timestamp = ack.timestamp;
			msg.target_system = ack.target_system;
			msg.operation_type = ack.operation_type;
			msg.result = ack.result;
			msg.old_value = ack.old_value;
			msg.new_value = ack.new_value;

			mavlink_msg_swarm_operation_ack_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // SWARM_OPERATION_ACK_HPP

