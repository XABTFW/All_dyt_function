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

uint8 state
uint8 requested_submode
uint8 active_submode
uint8 lost_reason

bool active
bool target_locked
bool controlling_vehicle   # true when this module owns the trajectory/offboard setpoints (aircraft motion)
bool target_fresh
bool intercept_allowed

float32 los_age_s
float32 frame_dt_s
float32 delay_s

float32[3] los_ned
float32[3] omega_los_ned
float32[3] velocity_sp
float32[3] acceleration_sp

float32 yaw_sp
float32 yaw_rate_sp
