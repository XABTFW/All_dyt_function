uint64 timestamp  # publication time (microseconds)
uint64 timestamp_sample  # range sample time (microseconds)

uint32 device_id
float32 distance_m
float32 closing_speed_m_s  # filtered speed; positive means approaching the target
bool valid
