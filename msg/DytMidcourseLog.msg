uint64 timestamp
uint64 target_timestamp
uint64 command_timestamp

uint32 target_id
uint8 target_source
bool gps_valid
bool pointing_valid

float32 target_age_s
float64 target_lat_deg
float64 target_lon_deg
float64 target_alt_m
float32[3] target_velocity_ned_m_s
float32[3] target_position_ned_m
float32[3] los_ned_m

# Frame angles after mount/sign/zero-offset correction. The unconstrained
# values are before frame-limit clamping; *_sp_deg is the command after it.
float32 frame_yaw_unconstrained_deg
float32 frame_pitch_unconstrained_deg
float32 frame_yaw_sp_deg
float32 frame_pitch_sp_deg
