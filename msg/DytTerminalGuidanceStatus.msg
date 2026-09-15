uint64 timestamp

# Timing, estimator, limiter and high-speed lock-loss diagnostics for the
# feature-gated terminal guidance law.
uint64 los_receive_timestamp
uint64 los_effective_timestamp
uint64 gimbal_effective_timestamp

bool enabled
bool los_rate_valid
bool los_rate_limited
bool acceleration_limited
bool jerk_limited
bool loss_coast_active

uint32 los_reject_count
float32 los_step_rad
float32 effective_k_omega_m_s
float32 course_rate_sp_rad_s

float32[3] los_raw_ned
float32[3] omega_los_raw_ned
float32[3] omega_los_filtered_ned
float32[3] guidance_accel_raw_ned
float32[3] guidance_accel_limited_ned
