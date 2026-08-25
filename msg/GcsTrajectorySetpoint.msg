uint64 timestamp # time since system start (microseconds)

# Ground-station position setpoint in the local NED frame.
float32[3] position # meters
float32[3] velocity # meters/second, NaN when not provided
float32 yaw # radians, NaN when not provided
