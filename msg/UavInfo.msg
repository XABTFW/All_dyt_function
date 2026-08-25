uint64 timestamp  # time since system start (microseconds)

uint32 mavid      #mavid
uint32 group_id   # swarm group id (for multi-group formation)
bool is_leader    # true if this aircraft is a leader in its group
float64 lat       # uav position (deg * 1e7)
float64 lon       # uav position (deg * 1e7)
float64 alt       # uav altitude (m)
float64 vx	  # uav velocity of vx
float64 vy	  # uav velocity of vy
float64 vz	  # uav velocity of vz
float64 yaw	  # uav yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
float64 yawspeed  # uav yawspeed (only for multirotors, in rad/s)
uint32 land       # true if uav landing
uint8 at_target   # 1 if aircraft has reached target position (for collision avoidance priority)
