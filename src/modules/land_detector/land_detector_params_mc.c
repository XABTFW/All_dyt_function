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
 * Multicopter land detection trigger time
 *
 * Total time it takes to go through all three land detection stages:
 * ground contact, maybe landed, landed
 * when all necessary conditions are constantly met.
 *
 * @unit s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_TRIG_TIME, 1.0f);

/**
 * Multicopter vertical velocity threshold
 *
 * Vertical velocity threshold to detect landing.
 * Has to be set lower than the expected minimal speed for landing,
 * which is either MPC_LAND_SPEED or MPC_LAND_CRWL.
 * This is enforced by an automatic check.
 *
 * @unit m/s
 * @min 0
 * @decimal 2
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_Z_VEL_MAX, 0.25f);

/**
 * Multicopter max horizontal velocity
 *
 * Maximum horizontal velocity allowed in the landed state
 *
 * @unit m/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_XY_VEL_MAX, 1.5f);

/**
 * Multicopter max rotational speed
 *
 * Maximum allowed norm of the angular velocity (roll, pitch) in the landed state.
 *
 * @unit deg/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_ROT_MAX, 20.0f);

/**
 * Ground effect altitude for multicopters
 *
 * The height above ground below which ground effect creates barometric altitude errors.
 * A negative value indicates no ground effect.
 *
 * @unit m
 * @min -1
 * @decimal 2
 * @group Land Detector
 *
 */
PARAM_DEFINE_FLOAT(LNDMC_ALT_GND, 2.f);

/**
 * Enable ToF fast touchdown detection
 *
 * When enabled, fresh downward-facing laser range samples can accelerate the
 * multicopter landed-state transition during an automatic LAND setpoint.
 * Keep disabled until the sensor and trigger distance have been validated.
 *
 * @boolean
 * @group Land Detector
 */
PARAM_DEFINE_INT32(LNDMC_TD_EN, 0);

/**
 * ToF fast touchdown trigger distance
 *
 * This is the sensor-to-ground measurement, not vehicle ground clearance.
 * It must be calibrated for the sensor mounting height and landing surface.
 *
 * @unit m
 * @min 0.05
 * @max 1.00
 * @decimal 2
 * @increment 0.01
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_TD_DIST, 0.28f);

/**
 * ToF fast touchdown independent altitude limit
 *
 * Maximum barometric/local height above Home at which fast touchdown may use
 * range data. A valid local Home position is required. This is independent of
 * the range measurement, so an implausibly low ToF reading above this height
 * cannot trigger landed. Landings more than 0.5 m below Home elevation use the
 * normal PX4 land detector instead.
 *
 * @unit m
 * @min 0.20
 * @max 10.00
 * @decimal 1
 * @increment 0.5
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_TD_ALT, 5.0f);

/**
 * ToF fast touchdown operational range
 *
 * Maximum distance at which measurements are considered usable for the
 * fast-touchdown detection. Samples above this value are rejected.
 *
 * @unit m
 * @min 0.50
 * @max 10.00
 * @decimal 1
 * @increment 0.1
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_TD_MAX, 5.0f);

/**
 * ToF fast touchdown confirmation time
 *
 * At least three distinct range samples are always required in addition to
 * this minimum elapsed time.
 *
 * @unit s
 * @min 0.02
 * @max 0.20
 * @decimal 2
 * @increment 0.01
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_TD_TIME, 0.04f);
