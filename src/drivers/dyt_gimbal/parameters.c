/**
 * DYT serial baudrate
 *
 * @min 9600
 * @max 921600
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_BAUD, 115200);

/**
 * DYT telemetry timeout
 *
 * @unit ms
 * @min 50
 * @max 5000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_TO_MS, 300);

/**
 * DYT port reopen retry interval
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_RTRY_MS, 1000);

/**
 * DYT debug log period
 *
 * Set to 0 to disable periodic decoded-frame logs. When greater than 0,
 * the driver prints one parsed target sample to the system log every N ms,
 * which can be read from the NSH `dmesg` command.
 *
 * @unit ms
 * @min 0
 * @max 5000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_LOG_MS, 0);

/**
 * DYT raw frame logging
 *
 * Set to 1 to print the original telemetry/reply frame bytes in hex.
 * Telemetry hex logs follow the same throttle as DYT_LOG_MS, while command
 * replies are printed whenever they are received.
 *
 * @boolean
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_RAWLOG, 0);

/**
 * Tweety V2 manual pixel-angle scale
 *
 * Optional fixed conversion from one tracking miss-distance pixel to degrees.
 * Set to 0 to derive the conversion from the live horizontal field of view and
 * the configured image dimensions. A positive value overrides that dynamic
 * conversion and should only be used for a fixed, calibrated zoom setting.
 *
 * @min 0
 * @max 1
 * @decimal 6
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_LOS_SC, 0.f);

/**
 * Visible tracking image width
 *
 * Width of the visible-light image used by the tracker. The Tweety V2.0.9.6
 * tracking parameter table specifies a visible-light range up to 1920 pixels.
 *
 * @min 16
 * @max 8192
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_VIS_W, 1920);

/**
 * Visible tracking image height
 *
 * Height of the visible-light image used by the tracker. The Tweety V2.0.9.6
 * tracking parameter table specifies a visible-light range up to 1080 pixels.
 *
 * @min 16
 * @max 8192
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_VIS_H, 1080);

/**
 * Infrared tracking image width
 *
 * Width of the infrared image used by the tracker. The Tweety V2.0.9.6
 * tracking parameter table specifies an infrared range up to 1280 pixels.
 *
 * @min 16
 * @max 8192
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_IR_W, 1280);

/**
 * Infrared tracking image height
 *
 * Height of the infrared image used by the tracker. The Tweety V2.0.9.6
 * tracking parameter table specifies an infrared range up to 1024 pixels.
 *
 * @min 16
 * @max 8192
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_IR_H, 1024);

/**
 * Tweety V2 locked tracking value
 *
 * Raw value in byte 32 of the V2 servo status frame that represents a valid
 * locked target. The protocol document defines the field as a char but does
 * not list its enumeration.
 *
 * @min 0
 * @max 255
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_TRK_VAL, 1);

/**
 * DYT startup home command enable
 *
 * Sends frame-angle commands after the serial link opens so the gimbal moves
 * from its power-on centered position to the configured startup view.
 *
 * @boolean
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_EN, 1);

/**
 * DYT startup home command delay
 *
 * Delay after opening the serial link before sending the startup frame-angle
 * command. This gives the gimbal time to finish its own power-on centering.
 *
 * @unit ms
 * @min 0
 * @max 30000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_DLY, 3000);

/**
 * DYT startup home command duration
 *
 * Duration over which the startup frame-angle command is resent. Repeating the
 * command helps if the gimbal is still completing its own power-on sequence.
 *
 * @unit ms
 * @min 0
 * @max 30000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_DUR, 10000);

/**
 * DYT startup home command interval
 *
 * Interval between repeated startup frame-angle commands.
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_HOME_INT, 500);

/**
 * DYT startup home yaw angle
 *
 * Frame yaw angle sent by the startup home command.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_HOME_YAW, 0.f);

/**
 * DYT startup home pitch angle
 *
 * Frame pitch angle sent by the startup home command. Positive pitch is up for
 * the DYT frame-angle command.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_HOME_PIT, 90.f);

/**
 * DYT geo-track roll sign
 *
 * Sign applied to vehicle roll before sending the DYT ownship state frame used
 * by geographic tracking. Use this to compensate payload installation
 * conventions without changing aircraft control.
 *
 * @value -1 Invert
 * @value 1 Normal
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_GEO_RSIGN, 1);

/**
 * DYT geo-track pitch sign
 *
 * Sign applied to vehicle pitch before sending the DYT ownship state frame used
 * by geographic tracking. Set to -1 if geographic tracking drives pitch in the
 * opposite direction for the payload installation.
 *
 * @value -1 Invert
 * @value 1 Normal
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_GEO_PSIGN, 1);

/**
 * DYT geo-track yaw sign
 *
 * Sign applied to vehicle yaw before sending the DYT ownship state frame used
 * by geographic tracking.
 *
 * @value -1 Invert
 * @value 1 Normal
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_GEO_YSIGN, 1);

/**
 * DYT geo-track roll offset
 *
 * Offset added after DYT_GEO_RSIGN is applied to vehicle roll in the ownship
 * state frame used by geographic tracking.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_ROFF, 0.f);

/**
 * DYT geo-track pitch offset
 *
 * Offset added after DYT_GEO_PSIGN is applied to vehicle pitch in the ownship
 * state frame used by geographic tracking. For an upward-looking installation,
 * try +/-90 degrees only after first checking the sign.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_POFF, 0.f);

/**
 * DYT geo-track yaw offset
 *
 * Offset added after DYT_GEO_YSIGN is applied to vehicle yaw in the ownship
 * state frame used by geographic tracking.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_YOFF, 0.f);

/**
 * DYT geo-track 0x72 pitch offset
 *
 * Offset added directly to the computed pitch angle before sending the new
 * protocol 0x72 geographic pointing command. Positive values command the
 * seeker farther upward. This does not change ownship attitude frame 0x11.
 *
 * @unit deg
 * @min -45
 * @max 45
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_CMD_POFF, 0.f);

/**
 * DYT geo-track mount rotation enable
 *
 * Enables a full Euler rotation from gimbal installation frame to aircraft body
 * frame before sending the ownship attitude used by geographic tracking. Keep
 * disabled to use the legacy per-axis sign and offset correction.
 *
 * @boolean
 * @group DYT Gimbal
 */
PARAM_DEFINE_INT32(DYT_GEO_MNT_EN, 0);

/**
 * DYT geo-track mount roll
 *
 * Roll angle of the gimbal installation frame relative to the aircraft body
 * frame. Applied as part of a full Euler rotation when DYT_GEO_MNT_EN is set.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_MNT_R, 0.f);

/**
 * DYT geo-track mount pitch
 *
 * Pitch angle of the gimbal installation frame relative to the aircraft body
 * frame. For an installation where gimbal +X points along aircraft -Z, start
 * with +90 or -90 degrees and verify the direction on the ground.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_MNT_P, 0.f);

/**
 * DYT geo-track mount yaw
 *
 * Yaw angle of the gimbal installation frame relative to the aircraft body
 * frame. Applied as part of a full Euler rotation when DYT_GEO_MNT_EN is set.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group DYT Gimbal
 */
PARAM_DEFINE_FLOAT(DYT_GEO_MNT_Y, 0.f);
