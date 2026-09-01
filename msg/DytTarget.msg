uint64 timestamp  # time since system start (microseconds)
uint64 timestamp_sample  # telemetry frame receive time (microseconds)

uint8 TRACKING_STATE_SEARCH = 0
uint8 TRACKING_STATE_LOCKED = 1
uint8 TRACKING_STATE_TIMEOUT = 2
uint8 TRACKING_STATE_ERROR = 3

uint8 VIDEO_SOURCE_VIS_1 = 0
uint8 VIDEO_SOURCE_VIS_2 = 1
uint8 VIDEO_SOURCE_IR_1 = 2
uint8 VIDEO_SOURCE_IR_2 = 3

uint8 TRACK_ALGO_ADAPTIVE = 0
uint8 TRACK_ALGO_PERSON = 1
uint8 TRACK_ALGO_VEHICLE = 2
uint8 TRACK_ALGO_BUILDING = 3

uint32 frame_counter

uint8 tracking_state
uint8 video_source
uint8 tracking_algorithm

bool target_valid
bool auto_hint
bool image_enhance
bool recording
bool motor_on
bool follow_mode
bool laser_on

bool selftest_done
bool gyro_calib_failed
bool servo_fault
bool image_board_fault

float32 los_x_rad
float32 los_y_rad

float32 gimbal_roll_rad
float32 gimbal_pitch_frame_rad  # pitch frame angle, protocol bytes 12-13
float32 gimbal_pitch_rad  # pitch attitude angle, protocol bytes 24-25
float32 gimbal_yaw_rad

float32 gimbal_roll_rate_rad_s
float32 gimbal_pitch_rate_rad_s
float32 gimbal_yaw_rate_rad_s

float32 bbox_width_px  # target box width, servo-status protocol bytes 40-41 (big-endian)
float32 bbox_height_px # target box height, servo-status protocol bytes 42-43 (big-endian)
float32 range_m
float32 zoom_ratio

float32 frame_dt_s
float32 last_rx_age_s

uint16 parse_error_count
uint8 status1
uint8 status2
uint8 status3
uint8 self_test_raw
