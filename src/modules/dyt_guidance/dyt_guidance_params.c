/**
 * DYT aircraft role
 *
 * Identifies the aircraft mission type reported to the ground station.
 * A net-capture aircraft changes this parameter to impact aircraft after the
 * net-capture braking sequence completes, regardless of guidance control mode.
 *
 * @value 1 Impact aircraft
 * @value 2 Net-capture aircraft
 * @value 3 Target aircraft (test only)
 * @min 1
 * @max 3
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYT_VEH_TYPE, 2);

/**
 * Terminal guidance activation AUX channel
 *
 * Enables DYT visual guidance to take over aircraft motion after seeker lock.
 * Midcourse geographic pointing follows the Cooperative Rendezvous activation
 * switch (CRDZ_ACT_AUX / CRDZ_ACT_BTN) when DYTG_COOP_EN is enabled.
 *
 * @value 0 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ACT_AUX, 1);

/**
 * Terminal guidance activation joystick button
 *
 * Enables DYT visual guidance to take over aircraft motion after seeker lock
 * using the MANUAL_CONTROL buttons bitmask. Button numbers match the zero-based
 * numbering shown by QGroundControl. Midcourse geographic pointing follows the
 * Cooperative Rendezvous activation button (CRDZ_ACT_BTN) when DYTG_COOP_EN is
 * enabled.
 *
 * @value -1 Disabled
 * @min -1
 * @max 15
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ACT_BTN, -1);

/**
 * Intercept AUX channel
 *
 * @value 0 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_INT_AUX, 2);

/**
 * Cooperative rendezvous handoff enable
 *
 * When enabled, the DYT seeker only commands aircraft motion (trajectory /
 * offboard setpoints) while the camera is locked and tracking. While searching
 * or after losing the lock, the seeker controls the gimbal only and leaves the
 * aircraft motion to the cooperative_rendezvous position-sharing follower, so
 * the two controllers never publish setpoints at the same time. The
 * Cooperative Rendezvous activation switch also enables midcourse geographic
 * pointing, so the payload can look at the shared target before terminal
 * guidance is authorized.
 *
 * Disable for standalone seeker operation (the seeker then holds position while
 * searching, as before).
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_COOP_EN, 1);

/**
 * Midcourse geographic tracking enable
 *
 * When enabled, midcourse pointing uses the DYT payload geographic tracking
 * protocol with ownship state and target latitude/longitude/altitude packets.
 * Disable to compute a frame-angle command in PX4 from the shared target
 * position instead. Disabling avoids sending near-vertical ownship Euler
 * attitudes for upward-looking payload installations.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_GEO_EN, 1);

/**
 * Midcourse target MAV_SYS_ID
 *
 * Target aircraft ID used to point the seeker before visual lock. Set to 0 to
 * use the newest valid remote follower_info sample that is not this vehicle.
 *
 * @min 0
 * @max 255
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_TGT_ID, 1);

/**
 * Midcourse target position timeout
 *
 * Maximum age of position-sharing target data used to point the seeker before
 * visual lock.
 *
 * @unit s
 * @min 0.1
 * @max 30.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_TGT_TO, 2.0f);

/**
 * Midcourse target altitude mode
 *
 * Selects how follower_info.alt is interpreted for midcourse geographic
 * pointing. Mode 0 keeps the legacy AMSL behavior. Mode 1 estimates target
 * relative altitude from the target's lowest observed altitude in this boot,
 * unless the incoming altitude already looks relative. Mode 2 treats
 * follower_info.alt as relative altitude directly.
 *
 * @value 0 Legacy AMSL
 * @value 1 Relative reference
 * @value 2 Incoming relative
 * @min 0
 * @max 2
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ALT_MODE, 1);

/**
 * Midcourse target altitude offset
 *
 * Offset added to the midcourse target height before sending the geographic
 * target to the DYT seeker. In relative altitude modes this is added to the
 * target-relative-minus-own-relative height.
 *
 * @unit m
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_TGT_ALTOFF, 0.0f);

/**
 * Midcourse gimbal prediction time
 *
 * Prediction time used when PX4 computes midcourse gimbal frame-angle commands
 * from ownship and target positions. This compensates seeker/gimbal response
 * delay by pointing at the predicted line of sight. Set to 0 to disable.
 *
 * @unit s
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_PRED, 0.0f);

/**
 * Midcourse gimbal angle mode
 *
 * Selects how PX4 computes midcourse frame-angle commands. Mode 0 keeps the
 * legacy body/mount-relative yaw and pitch. Mode 1 keeps the legacy
 * body/mount-relative yaw but uses inertial/horizon LOS elevation for pitch.
 *
 * @value 0 Body frame
 * @value 1 Inertial pitch only
 * @min 0
 * @max 1
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_MNT_MODE, 1);

/**
 * Manual takeover stick threshold
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_STK_TK, 0.30f);

/**
 * DYT target-control mode
 *
 * Manual mode disables automatic target activation. In semi-automatic mode,
 * selecting a target at the ground station authorizes terminal guidance as soon
 * as the payload reports a fresh lock. Full automatic mode enables automatic
 * target activation after the recognition hold time.
 *
 * @value 0 Manual
 * @value 1 Semi-automatic
 * @value 2 Full automatic
 * @min 0
 * @max 2
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_MODE, 0);

/**
 * Enable automatic guidance activation from detected target hints
 *
 * Compatibility mirror controlled by DYTG_MODE. It is zero in manual and
 * semi-automatic modes and one in full automatic mode. When enabled, no
 * activation AUX/button is required. After recognition value
 * 100 remains continuously valid for 0.4 seconds, guidance starts a
 * DYTG_LOCK_MS lock window and sends a 0x06 request every 500 ms until the
 * payload reports that tracking acquisition is in progress. A failed window
 * ends tracking, restarts detection, and allows persistent recognition to
 * qualify for another attempt after a new 0.4-second hold.
 * Selecting another flight mode exits the automatic terminal-guidance session.
 * An explicit activation AUX/button rising edge continues to request tracking
 * directly, independently of payload recognition.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_AUTO_EN, 0);

/**
 * Lock request protection time
 *
 * Time reserved for the payload to complete a 0x06 lock request. During this
 * interval, pointing or search commands do not replace the tracking request.
 * The search-wait timeout is never shorter than this value.
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LOCK_MS, 2000);

/**
 * Search wait timeout
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_WAITMS, 1500);

/**
 * Lost hold/search timeout
 *
 * @unit ms
 * @min 100
 * @max 600000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LOSTMS, 240000);

/**
 * Search yaw speed
 *
 * Gimbal yaw rate used while scanning after target loss.
 *
 * @unit deg/s
 * @min 1
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_YSPD, 3.f);

/**
 * Search pitch speed
 *
 * Gimbal pitch rate used while changing scan rows after target loss.
 *
 * @unit deg/s
 * @min 1
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_PSPD, 3.f);

/**
 * Search pitch step
 *
 * @unit deg
 * @min 1
 * @max 30
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_STEP, 20.f);

/**
 * Search edge pause
 *
 * @unit s
 * @min 0
 * @max 2
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_PAUSE, 0.05f);

/**
 * Search yaw step
 *
 * @unit deg
 * @min 5
 * @max 60
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_YSTP, 10.f);

/**
 * Search dwell time
 *
 * @unit s
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SC_DWEL, 0.2f);

/**
 * Gimbal center settle time before relock retry
 *
 * @unit ms
 * @min 0
 * @max 5000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_CTRMS, 400);

/**
 * Lost target relock retry interval
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_RTRYMS, 1000);

/**
 * Fixed image pipeline delay
 *
 * Estimated time from image exposure/tracker sampling until the corresponding
 * servo-status frame is received by PX4. The payload protocol has no sample
 * timestamp, so DYT guidance subtracts this delay from the receive time before
 * interpolating aircraft attitude and converting the body LOS to NED.
 *
 * @unit ms
 * @min 0
 * @max 1000
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_DLY_MS, 30.f);

/**
 * Gimbal telemetry delay
 *
 * Estimated delay from sampling the gimbal attitude until the servo-status
 * frame is received. This is calibrated separately from the image delay.
 *
 * @unit ms
 * @min 0
 * @max 1000
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_GMB_DLY, 0.f);

/**
 * Terminal guidance law
 *
 * @value 0 Legacy LOS/PN control
 * @value 1 Bounded fixed-speed LOS turn control
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_GD_LAW, 0);

/**
 * Terminal LOS-rate steering enable
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_PN_EN, 0);

/**
 * Terminal acceleration feedforward enable
 *
 * When disabled, the bounded acceleration is used to generate the velocity
 * setpoint but is not sent to the PX4 velocity controller as feedforward.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ACC_FF, 0);

/**
 * Terminal LOS filter time constant
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_LOS_TC, 0.08f);

/**
 * Terminal LOS-rate filter time constant
 *
 * @unit s
 * @min 0.01
 * @max 2.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_OMG_TC, 0.15f);

/**
 * Maximum accepted raw LOS rate
 *
 * Samples above this limit do not contribute a LOS-rate steering command.
 *
 * @unit rad/s
 * @min 0.05
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_OMG_MAX, 0.8f);

/**
 * Follow LOS-rate turn gain
 *
 * @unit m/s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KW_FOL, 10.f);

/**
 * Intercept LOS-rate turn gain
 *
 * @unit m/s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KW_INT, 10.f);

/**
 * Terminal horizontal acceleration jerk limit
 *
 * @unit m/s^3
 * @min 0.1
 * @max 30.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ACC_JERK, 4.f);

/**
 * Terminal horizontal speed slew rate
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_SPD_SLEW, 2.f);

/**
 * Maximum speed for entering position hold after terminal lock loss
 *
 * Above this speed the new terminal guidance law keeps the current course and
 * reduces the velocity setpoint at DYTG_SPD_SLEW instead of commanding an
 * immediate position hold.
 *
 * @unit m/s
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_HOLD_V, 2.f);

/**
 * Maximum target age
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXAGE, 0.25f);

/**
 * Maximum allowed frame gap
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXJIT, 0.12f);

/**
 * Terminal handoff blend time
 *
 * Time used to blend from the previous midcourse velocity setpoint to the
 * visual guidance velocity setpoint after seeker lock. Set to 0 to disable the
 * blend.
 *
 * @unit s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_HOFF_T, 0.80f);

/**
 * Maximum delay allowed for intercept
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_INTDLY, 0.18f);

/**
 * Follow navigation gain
 *
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_N_FOL, 2.0f);

/**
 * Follow commanded speed
 *
 * @unit m/s
 * @min 0.1
 * @max 60.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_V_FOL, 2.0f);

/**
 * Follow LOS acceleration gain
 *
 * @unit m/s^2
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KA_FOL, 1.2f);

/**
 * Follow damping gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KV_FOL, 1.0f);

/**
 * Intercept navigation gain
 *
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_N_INT, 3.5f);

/**
 * Intercept commanded speed
 *
 * @unit m/s
 * @min 0.1
 * @max 60.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_V_INT, 6.0f);

/**
 * Intercept LOS acceleration gain
 *
 * @unit m/s^2
 * @min 0.0
 * @max 25.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KA_INT, 2.0f);

/**
 * Intercept damping gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_KV_INT, 1.2f);

/**
 * Minimum closing speed proxy
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_VMIN, 1.0f);

/**
 * Maximum horizontal speed setpoint
 *
 * @unit m/s
 * @min 0.5
 * @max 60.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXV, 8.0f);

/**
 * Maximum acceleration feedforward
 *
 * @unit m/s^2
 * @min 0.5
 * @max 30.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXACC, 4.0f);

/**
 * Legacy net release minimum range
 *
 * Retained for compatibility with the former laser-distance release strategy.
 * It is not used by the target-box automatic release strategy.
 *
 * @unit m
 * @min 0.0
 * @max 50.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_RNG_MIN, 0.f);

/**
 * Net release near-range diagnostic threshold
 *
 * Retained as the laser near-range threshold for attitude diagnostics. It does
 * not trigger target-box automatic release.
 *
 * @unit m
 * @min 0.0
 * @max 50.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_RNG_MAX, 0.f);

/**
 * Net release pitch action duration
 *
 * Maximum duration of the line-of-sight alignment action after the manual or
 * fused-distance trigger. Automatic release ends this action at the configured
 * timeout or 50 ms after the gripper PWM command, whichever is earlier. During
 * the action the attitude correction is recomputed every control cycle. Set to
 * 0 to disable the attitude action; automatic release then uses only the final
 * fused-distance threshold.
 *
 * @unit ms
 * @min 0
 * @max 500
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_SZ_MS, 500);

/**
 * Adaptive net release pitch gain
 *
 * Gain applied to target line-of-sight elevation minus body -Z launch-axis
 * elevation. The net-capture calculation intentionally excludes DYTG_POFF and
 * DYTG_YOFF, while retaining the configured gimbal-to-body mount rotation. A
 * negative value reverses the correction direction.
 *
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ALP_K, 1.0f);

/**
 * Maximum net release pitch correction
 *
 * Limits the equivalent nose-up or nose-down correction generated before the
 * gripper PWM release. The absolute value is used as the limit.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ALP_MAX, 10.0f);

/**
 * Legacy minimum speed for net release pitch correction
 *
 * Retained for parameter compatibility. The image-triggered line-of-sight
 * alignment does not use vehicle ground speed and ignores this parameter.
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ALP_VMIN, 1.0f);

/**
 * Manual net release AUX channel
 *
 * Starts the adaptive pitch action and delayed gripper release sequence. This
 * bypasses the fused-distance thresholds and is intended for manual visual
 * confirmation of the target.
 *
 * @value -1 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_FIRE_AUX, -1);

/**
 * Manual net release joystick button
 *
 * Starts the manual adaptive pitch action and delayed gripper release sequence,
 * bypassing the fused-distance thresholds. Button numbers match the zero-based
 * numbering shown by QGroundControl.
 *
 * @value -1 Disabled
 * @min -1
 * @max 15
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_FIRE_BTN, -1);

/**
 * Enable fused target-range net release
 *
 * A fresh locked visible-light or infrared target at zoom 1.0 is always
 * required. The long-side image estimate uses the active source resolution and
 * provides continuity, while fresh gated SDM50 range and closing speed calibrate
 * it. During laser dropouts the calibrated image estimate is used. Attitude
 * alignment starts when fused range is less than fused closing speed * 0.3 s
 * + DYTG_FIRE_D. The gripper PWM is sent when fused range is less than fused
 * closing speed * 0.05 s + DYTG_FIRE_D. The area fit remains
 * diagnostic only.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_FIRE_EN, 1);

/**
 * Enable laser/image range fusion
 *
 * Uses fresh gated SDM50 distance to update a multiplicative image-distance
 * correction and uses fresh gated SDM50 closing speed to update a multiplicative
 * image-speed correction. Set to zero to retain the original image-only trigger.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_FUS_EN, 1);

/**
 * Net release base trigger distance
 *
 * Base distance added to the fused closing-speed lookaheads. Attitude alignment
 * starts at fused closing speed * 0.3 s plus DYTG_FIRE_D, and the gripper PWM is
 * sent at fused closing speed * 0.05 s plus DYTG_FIRE_D.
 *
 * @unit m
 * @min 0.1
 * @max 12.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_FIRE_D, 2.0f);

/**
 * Enable net release hold
 *
 * When enabled, DYT guidance captures the current local position and switches
 * to position hold immediately after a gripper release command. Disable to keep
 * using the post-release low-speed tracking behavior. The automatic recovery
 * after an unreleased 500 ms attitude timeout always performs its required
 * brake-and-hold sequence independently of this parameter.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_HOLD_EN, 1);

/**
 * Net release stop distance
 *
 * Target horizontal stopping distance used by the adaptive braking phase after
 * net release. The commanded braking acceleration is calculated from the current
 * horizontal speed and this distance, then limited by DYTG_STOP_ACC and
 * DYTG_MAXACC.
 *
 * @unit m
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_STOP_D, 0.5f);

/**
 * Net release hold speed threshold
 *
 * Horizontal speed below which the adaptive braking phase switches to position
 * hold after net release.
 *
 * @unit m/s
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_STOP_V, 0.2f);

/**
 * Net release stop acceleration
 *
 * Maximum horizontal braking acceleration used by the adaptive braking phase
 * after net release. The effective value is still limited by DYTG_MAXACC.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_STOP_ACC, 4.0f);

/**
 * Enable net release deceleration
 *
 * When enabled, DYT guidance listens for gripper release commands and applies
 * a short reverse horizontal acceleration while reducing the visual tracking
 * horizontal velocity setpoint. This is intended for net-capture recovery after
 * the payload release command is sent.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_NET_EN, 1);

/**
 * Net release deceleration time
 *
 * Time window used to brake after a gripper release command.
 *
 * @unit ms
 * @min 0
 * @max 5000
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_NET_MS, 500);

/**
 * Net release speed scale
 *
 * Horizontal tracking speed scale held after a gripper release command. A value
 * of 0.5 commands half of the normal DYT horizontal tracking speed after the
 * release.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_NET_SC, 0.5f);

/**
 * Net release braking acceleration
 *
 * Maximum reverse horizontal acceleration added after a gripper release command.
 * The actual braking acceleration is reduced when less is needed to reach the
 * configured speed scale by DYTG_NET_MS.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_NET_ACC, 4.0f);

/**
 * Maximum yaw rate
 *
 * @unit deg/s
 * @min 5
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXYAWR, 30.f);

/**
 * Maximum yaw lag
 *
 * @unit deg
 * @min 1
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YAWLIM, 25.f);

/**
 * Maximum vertical speed magnitude
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MAXDZ, 1.0f);

/**
 * Vertical tracking scale
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ZSCALE, 0.25f);

/**
 * Enable vertical-priority XY scaling
 *
 * When enabled, horizontal tracking is reduced while the target is far above
 * or below the vehicle LOS. This helps avoid flying past the target projection
 * before vertical error has reduced.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_ZXY_EN, 0);

/**
 * Minimum XY scale during vertical-priority tracking
 *
 * Horizontal velocity and horizontal feedforward acceleration are never scaled
 * below this fraction when vertical-priority XY scaling is enabled.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ZXY_MIN, 0.25f);

/**
 * Vertical LOS value for maximum XY reduction
 *
 * When vertical-priority XY scaling is enabled, XY reduction reaches
 * DYTG_ZXY_MIN once abs(los_ned[2]) reaches this value.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ZXY_FULL, 0.70f);

/**
 * Enable XY overshoot guard
 *
 * When enabled, horizontal tracking is reduced if the vehicle is moving away
 * from the target horizontal LOS while the target is still far above or below.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_XYOVR_EN, 0);

/**
 * Vertical LOS threshold for XY overshoot guard
 *
 * XY overshoot protection starts once abs(los_ned[2]) is above this value.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYOVR_Z, 0.75f);

/**
 * Reverse horizontal closing speed for full XY overshoot guard
 *
 * XY overshoot protection reaches DYTG_XYOVR_MIN once the vehicle is moving
 * away from the target horizontal LOS by this speed.
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYOVR_V, 2.0f);

/**
 * Minimum XY scale during XY overshoot guard
 *
 * Horizontal velocity and horizontal feedforward acceleration are never scaled
 * below this fraction when XY overshoot protection is active.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYOVR_MIN, 0.15f);

/**
 * Enable XY LOS turn-rate guard
 *
 * When enabled, horizontal tracking is reduced before overshoot if horizontal
 * LOS direction is rotating quickly while the target is still far above or
 * below the vehicle.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_XYROT_EN, 0);

/**
 * Horizontal LOS turn rate for full XY guard
 *
 * The turn-rate guard reaches DYTG_XYROT_MIN when the horizontal LOS direction
 * rotates at this rate.
 *
 * @unit rad/s
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYROT_W, 1.0f);

/**
 * Minimum XY scale during LOS turn-rate guard
 *
 * Horizontal velocity and horizontal feedforward acceleration are never scaled
 * below this fraction when the turn-rate guard is active.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYROT_MIN, 0.20f);

/**
 * Horizontal LOS deadband near vertical target
 *
 * @min 0.0
 * @max 0.8
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYDB, 0.20f);

/**
 * Horizontal LOS value for full XY tracking
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYFULL, 0.60f);

/**
 * Minimum horizontal LOS for yaw tracking
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YAWLOS, 0.20f);

/**
 * Horizontal velocity setpoint slew rate
 *
 * @unit m/s^2
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_XYSLEW, 3.0f);

/**
 * Front cone half-angle for intercept
 *
 * @unit deg
 * @min 5
 * @max 90
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_FCONE, 35.f);

/**
 * LOS low-pass alpha
 *
 * @min 0.0
 * @max 0.99
 * @decimal 2
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_LPF_A, 0.65f);

/**
 * Maximum LOS prediction horizon
 *
 * @unit s
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_PREDMAX, 0.20f);

/**
 * LOS X sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LXSIGN, 1);

/**
 * LOS Y sign
 *
 * The DYT V2.11 telemetry reports vertical miss angle as down-positive and up-negative,
 * which already matches PX4 NED Z sign.
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_LYSIGN, 1);

/**
 * Gimbal roll sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_RSIGN, 1);

/**
 * Gimbal pitch sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_PSIGN, 1);

/**
 * Gimbal yaw sign
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_YSIGN, 1);

/**
 * Gimbal roll offset
 *
 * @unit deg
 * @min -180
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_ROFF, 0.f);

/**
 * Gimbal pitch offset
 *
 * @unit deg
 * @min -180
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_POFF, 0.f);

/**
 * Gimbal yaw offset
 *
 * @unit deg
 * @min -180
 * @max 180
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YOFF, 0.f);

/**
 * Gimbal frame yaw minimum
 *
 * Minimum yaw frame angle that DYT guidance may command.
 *
 * @unit deg
 * @min -180
 * @max 0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YAW_MIN, -40.f);

/**
 * Gimbal frame yaw maximum
 *
 * Maximum yaw frame angle that DYT guidance may command.
 *
 * @unit deg
 * @min 0
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_YAW_MAX, 40.f);

/**
 * Gimbal frame pitch minimum
 *
 * Minimum pitch frame angle that DYT guidance may command.
 *
 * @unit deg
 * @min -180
 * @max 0
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_PIT_MIN, -90.f);

/**
 * Gimbal frame pitch maximum
 *
 * Maximum pitch frame angle that DYT guidance may command.
 *
 * @unit deg
 * @min 0
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_PIT_MAX, 40.f);

/**
 * Midcourse mount rotation enable
 *
 * Enables gimbal installation compensation. Midcourse frame-angle commands use
 * the mount yaw as the horizontal alignment and the mount pitch as the camera
 * zero-elevation offset. Tracked target LOS conversion also applies the mount
 * installation rotation.
 *
 * @boolean
 * @group DYT Guidance
 */
PARAM_DEFINE_INT32(DYTG_MNT_EN, 0);

/**
 * Midcourse mount roll
 *
 * Roll angle of the gimbal installation frame relative to the aircraft body
 * frame when converting tracked target LOS. Applied when DYTG_MNT_EN is set.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_R, 0.f);

/**
 * Midcourse mount pitch
 *
 * Camera zero-elevation pitch relative to the aircraft body frame. For this
 * payload mounted with the camera pointing up at pitch zero and forward at
 * pitch -90 degrees, set +90 degrees.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_P, 0.f);

/**
 * Midcourse mount yaw
 *
 * Horizontal yaw alignment of the gimbal installation frame relative to the
 * aircraft body frame. Applied when DYTG_MNT_EN is set.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Guidance
 */
PARAM_DEFINE_FLOAT(DYTG_MNT_Y, 0.f);
