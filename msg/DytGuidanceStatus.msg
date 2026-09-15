uint64 timestamp  # time since system start (microseconds)

uint8 STATE_IDLE = 0
uint8 STATE_SEARCH_WAIT_LOCK = 1
uint8 STATE_TRACK_FOLLOW = 2
uint8 STATE_TRACK_INTERCEPT = 3
uint8 STATE_LOST_HOLD = 4
uint8 STATE_ABORT = 5

uint8 SUBMODE_FOLLOW = 0
uint8 SUBMODE_INTERCEPT = 1

uint8 LOST_REASON_NONE = 0
uint8 LOST_REASON_TRACKING = 1
uint8 LOST_REASON_TIMEOUT = 2
uint8 LOST_REASON_STALE = 3
uint8 LOST_REASON_MANUAL = 4
uint8 LOST_REASON_PRECONDITION = 5
uint8 LOST_REASON_JITTER = 6

uint8 VEHICLE_TYPE_FIGHTER = 1  # impact aircraft
uint8 VEHICLE_TYPE_NET_CAPTURE = 2
uint8 VEHICLE_TYPE_TARGET = 3   # test target aircraft

uint8 PHASE_DISARMED = 0
uint8 PHASE_INITIAL = 1
uint8 PHASE_MIDCOURSE = 2
uint8 PHASE_TERMINAL = 3

uint8 COMMAND_RESULT_NONE = 0
uint8 COMMAND_RESULT_PENDING = 1
uint8 COMMAND_RESULT_ACCEPTED = 2
uint8 COMMAND_RESULT_DENIED = 3
uint8 COMMAND_RESULT_FAILED = 4

uint8 CONTROL_MODE_MANUAL = 0
uint8 CONTROL_MODE_SEMI_AUTO = 1
uint8 CONTROL_MODE_FULL_AUTO = 2

uint8 SEMI_STATE_DISABLED = 0
uint8 SEMI_STATE_IDLE = 1
uint8 SEMI_STATE_LOCK_REQUESTED = 2
uint8 SEMI_STATE_LOCKED_WAIT_CONFIRM = 3
uint8 SEMI_STATE_GUIDANCE_ACTIVE = 4

uint32 command_sequence
uint32 net_trigger_count

uint8 vehicle_type
uint8 control_mode
uint8 semi_auto_state
uint8 guidance_phase
uint8 gcs_phase_request           # accepted GCS phase request controlling the modules, 0 if none
uint8 command_phase               # phase carried by the latest received GCS command
uint8 command_result

bool net_trigger_sent             # net-capture trigger command was sent in this terminal session
bool auto_midcourse_requested     # onboard height trigger is requesting midcourse guidance
bool midcourse_active
bool midcourse_target_valid

uint8 state
uint8 requested_submode
uint8 active_submode
uint8 lost_reason

bool active
bool target_locked
bool controlling_vehicle   # true when this module owns the trajectory/offboard setpoints (aircraft motion)
bool target_fresh
bool intercept_allowed

# Raw image-based net-release estimator. The long-side distance feeds the
# laser/image fusion; area distance and disagreement remain diagnostics.
# Positive speed means approaching.
bool image_range_valid
bool image_closing_speed_valid
float32 bbox_area_ratio
float32 image_distance_area_m
float32 image_distance_long_m
float32 image_distance_disagreement_m
float32 image_closing_speed_m_s
float32 image_trigger_distance_m

# Laser/image complementary fusion used by the automatic net-release trigger.
# Bias is raw image estimate minus accepted laser estimate.
bool fused_range_valid
bool fused_closing_speed_valid
bool laser_fusion_used
float32 fused_distance_m
float32 fused_closing_speed_m_s
float32 laser_distance_m
float32 laser_closing_speed_m_s
float32 image_distance_bias_m
float32 image_speed_bias_m_s

float32 los_age_s
float32 frame_dt_s
float32 delay_s

float32[3] los_ned
float32[3] omega_los_ned
float32[3] velocity_sp
float32[3] acceleration_sp

float32 yaw_sp
float32 yaw_rate_sp

# Shadow attitude-alignment diagnostic. These values are logged only and are
# never used as control setpoints. The desired attitude keeps the current yaw
# while aligning the body -Z axis with los_ned.
bool attitude_diag_valid
bool attitude_diag_near              # valid SDM50 range is at or below DYTG_RNG_MAX
float32 attitude_diag_range_m
float32 desired_roll_rad
float32 desired_pitch_rad
float32 actual_roll_rad
float32 actual_pitch_rad
float32 roll_error_rad
float32 pitch_error_rad
float32 attitude_error_rad            # shortest desired-to-actual quaternion angle
float32 max_abs_roll_error_rad        # maximum while attitude_diag_near
float32 max_abs_pitch_error_rad       # maximum while attitude_diag_near
float32 max_attitude_error_rad        # maximum while attitude_diag_near
