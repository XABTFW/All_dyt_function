/**
 * Activation AUX channel
 *
 * Controls whether the rendezvous aircraft may publish Offboard setpoints.
 * Position broadcast remains enabled while the module is running.
 *
 * @value 0 Always enabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_ACT_AUX, 3);

/**
 * Activation joystick button
 *
 * Controls whether the rendezvous aircraft may publish Offboard setpoints using
 * the MANUAL_CONTROL buttons bitmask. Button numbers match the zero-based
 * numbering shown by QGroundControl.
 *
 * @value -1 Disabled
 * @min -1
 * @max 15
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_ACT_BTN, -1);

/**
 * Horizontal distance from target
 *
 * Desired horizontal distance for the rendezvous aircraft relative to the
 * target aircraft. Set to 0 to track the target position without a horizontal
 * offset. Negative values keep the startup offset from -d/-x/-y.
 *
 * @unit m
 * @min -1
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_DIST, 0.f);

/**
 * Enable explicit horizontal offsets
 *
 * When enabled, CRDZ_X_OFF and CRDZ_Y_OFF directly define the target-relative
 * NED horizontal offset. CRDZ_DIST is ignored for horizontal offset generation.
 * Keep disabled to use the legacy CRDZ_DIST scaling of the startup -x/-y
 * offset direction.
 *
 * @boolean
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_XY_OFF_EN, 0);

/**
 * Target NED X offset
 *
 * Exact NED X offset from the target aircraft used when CRDZ_XY_OFF_EN is set.
 * Positive is North in the local NED frame.
 *
 * @unit m
 * @min -100
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_X_OFF, -5.f);

/**
 * Target NED Y offset
 *
 * Exact NED Y offset from the target aircraft used when CRDZ_XY_OFF_EN is set.
 * Positive is East in the local NED frame.
 *
 * @unit m
 * @min -100
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_Y_OFF, 0.f);

/**
 * Approach speed toward target position
 *
 * Horizontal closing speed added while the rendezvous aircraft has not reached
 * the desired target-relative position. Keep this above the target aircraft
 * speed when the follower needs to catch up.
 *
 * Set to 0 to use the startup -v argument.
 *
 * @unit m/s
 * @min 0
 * @max 80
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_APP_SPD, 4.f);

/**
 * Slowdown radius near target position
 *
 * The approach speed stays at CRDZ_APP_SPD while horizontal position error is
 * larger than this radius, then scales down linearly to avoid overshoot near
 * the desired target-relative position.
 *
 * @unit m
 * @min 0.5
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_SLOW_RAD, 5.f);

/**
 * Enable arrival hold
 *
 * Changes rendezvous behavior after the aircraft reaches the target-relative
 * position for CRDZ_HOLD_T seconds. If the target aircraft is slower than
 * CRDZ_HOLD_VEL, the module freezes the absolute setpoint and publishes zero
 * velocity feed-forward. If the target aircraft is faster, the module keeps
 * following the live target-relative setpoint and target velocity, but stops
 * adding approach speed. The latch releases when the live target-relative
 * setpoint or follower error moves farther than CRDZ_HOLD_REL.
 *
 * @boolean
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_HOLD_EN, 0);

/**
 * Arrival hold position mode
 *
 * Selects which position error axes must be inside their hold radius before
 * the rendezvous setpoint can be frozen. Target speed must still be below
 * CRDZ_HOLD_VEL.
 *
 * @value 0 Horizontal and vertical
 * @value 1 Horizontal only
 * @value 2 Vertical only
 * @min 0
 * @max 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_HOLD_MODE, 0);

/**
 * Arrival hold horizontal radius
 *
 * Horizontal error threshold used before the rendezvous setpoint can be frozen.
 *
 * @unit m
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_HOLD_HRAD, 1.5f);

/**
 * Arrival hold vertical radius
 *
 * Vertical error threshold used before the rendezvous setpoint can be frozen.
 *
 * @unit m
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_HOLD_VRAD, 1.5f);

/**
 * Arrival hold target speed
 *
 * Maximum target aircraft horizontal and vertical speed allowed for absolute
 * setpoint freeze. Faster targets use live target-relative following after
 * arrival instead of freezing the absolute setpoint.
 *
 * @unit m/s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_HOLD_VEL, 0.5f);

/**
 * Arrival hold dwell time
 *
 * Time that the aircraft must stay inside the position radius selected by
 * CRDZ_HOLD_MODE before entering arrival behavior.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_HOLD_T, 1.f);

/**
 * Arrival hold release radius
 *
 * Distance from the frozen setpoint at which live target motion releases the
 * hold latch and normal rendezvous tracking resumes.
 *
 * @unit m
 * @min 0.1
 * @max 50
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_HOLD_REL, 5.f);

/**
 * Horizontal velocity slew limit
 *
 * Limits how quickly the rendezvous horizontal velocity setpoint may change.
 * Set to 0 to disable the limit and keep the legacy direct target-velocity
 * feed-forward behavior.
 *
 * @unit m/s^2
 * @min 0
 * @max 20
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_VSLEW, 3.f);

/**
 * Target position low-pass time constant
 *
 * Applies a first-order low-pass filter to the target aircraft local NED
 * position before generating the rendezvous position and velocity setpoints.
 * This helps reject telemetry jitter that would otherwise make the follower
 * accelerate and decelerate repeatedly.
 *
 * Set to 0 to disable position filtering.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TPOS_TC, 1.f);

/**
 * Target velocity low-pass time constant
 *
 * Applies a first-order low-pass filter to the target aircraft NED velocity
 * feed-forward before it is added to the rendezvous approach velocity.
 *
 * Set to 0 to disable velocity filtering.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TVEL_TC, 0.5f);

/**
 * Target position jump limit
 *
 * Limits one accepted target-position measurement jump before the low-pass
 * filter. This is useful when the target telemetry occasionally reports a
 * bad GPS/global position sample.
 *
 * Set to 0 to disable jump limiting.
 *
 * @unit m
 * @min 0
 * @max 200
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TPOS_JMP, 7.f);

/**
 * Maximum vertical setpoint error
 *
 * Rejects a target altitude when the resulting local NED vertical setpoint is
 * farther than this distance from the current aircraft position. Rejected
 * targets hold the current position and are not used to enter Offboard mode.
 *
 * @unit m
 * @min 1
 * @max 500
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_ALT_ERR, 100.f);

/**
 * Altitude difference from target
 *
 * Desired altitude difference for the rendezvous aircraft relative to the
 * target aircraft. A positive value keeps the rendezvous aircraft above the
 * target aircraft.
 *
 * @unit m
 * @min -50
 * @max 50
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_ALT_DIFF, 0.f);

/**
 * Ground-station target priority enable
 *
 * When enabled, a valid ground-station trajectory target has priority over
 * the target-aircraft position while cooperative rendezvous is active.
 * Disable to ignore ground-station trajectory targets in this module.
 *
 * @boolean
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_GCS_EN, 1);

/**
 * GCS setpoint timeout
 *
 * Time after the last valid ground-station trajectory setpoint before the
 * rendezvous aircraft falls back to the target-aircraft position source.
 *
 * @unit s
 * @min 0.1
 * @max 5
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_GCS_TOUT, 0.6f);

/**
 * Target aircraft position timeout
 *
 * Maximum age of the latest target-aircraft position used for rendezvous.
 * When it expires, the rendezvous aircraft keeps publishing its current
 * position as an Offboard hold setpoint.
 *
 * @unit s
 * @min 0.1
 * @max 30
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TGT_TOUT, 2.f);

/**
 * Minimum target-link height protection enable
 *
 * Enables the CRDZ_MIN_HGT clamp for setpoints derived from the other
 * aircraft position.
 *
 * @boolean
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_MINH_EN, 1);

/**
 * Minimum target-link height
 *
 * Minimum height above the local reference for target-aircraft rendezvous
 * setpoints. This limit is only applied to setpoints derived from the other
 * aircraft position; ground-station trajectory setpoints are not constrained
 * by this parameter. Set to 0 to disable.
 *
 * @unit m
 * @min 0
 * @max 100
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_MIN_HGT, 0.f);

/**
 * Target history and tracking delay enable
 *
 * Enables storage of target-aircraft position history and delayed following
 * through CRDZ_HIST_T and CRDZ_TRK_DLY.
 *
 * @boolean
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_INT32(CRDZ_HIST_EN, 1);

/**
 * Target history duration
 *
 * Time span retained for target-aircraft position history. The effective
 * tracking delay cannot exceed this duration. Changing this parameter clears
 * the existing history.
 *
 * @unit s
 * @min 0.1
 * @max 10
 * @decimal 1
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_HIST_T, 2.f);

/**
 * Target-link track delay
 *
 * Delay applied to rendezvous setpoints derived from the other aircraft
 * position history. The current target data is still used for target-facing
 * references; only the aircraft follow setpoint is delayed. Set to 0 to
 * disable historical tracking.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @group Cooperative Rendezvous
 */
PARAM_DEFINE_FLOAT(CRDZ_TRK_DLY, 1.f);
