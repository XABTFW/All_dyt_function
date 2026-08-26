/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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

/**
 * Total flight time in microseconds
 *
 * Total flight time of this autopilot. Higher 32 bits of the value.
 * Flight time in microseconds = (LND_FLIGHT_T_HI << 32) | LND_FLIGHT_T_LO.
 *
 * @min 0
 * @volatile
 * @category system
 * @group Land Detector
 *
 */
PARAM_DEFINE_INT32(LND_FLIGHT_T_HI, 0);

/**
 * Total flight time in microseconds
 *
 * Total flight time of this autopilot. Lower 32 bits of the value.
 * Flight time in microseconds = (LND_FLIGHT_T_HI << 32) | LND_FLIGHT_T_LO.
 *
 * @min 0
 * @volatile
 * @category system
 * @group Land Detector
 *
 */
PARAM_DEFINE_INT32(LND_FLIGHT_T_LO, 0);

/**
 * ToF fast touchdown height below Home tolerance
 *
 * Maximum local height below Home at which fast touchdown may use range data.
 * Increase this when automatic Home altitude correction can move Home above the
 * actual landing surface. The automatic LAND setpoint, descent, fresh range and
 * multi-sample checks remain required.
 *
 * @unit m
 * @min 0.00
 * @max 10.00
 * @decimal 1
 * @increment 0.5
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_TD_BELOW, 0.5f);
