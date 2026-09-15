uint64 timestamp  # time since system start (microseconds)

uint32 mavid      #mavid
float64 lat       # follower position (deg * 1e7)
float64 lon       # follower position (deg * 1e7)
float64 alt       # follower altitude AMSL (m)
float64 vx	  	  # follower velocity of vx
float64 vy	  	  # follower velocity of vy
float64 vz	  	  # follower velocity of vz
float64 yaw	  	  # follower yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
float64 yawspeed  # follower yawspeed (only for multirotors, in rad/s)
uint32 land       # ture if leader landing
uint8 at_target   # 1 if follower has reached target position (for collision avoidance priority)

uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_REAL_POSITION = 1
uint8 SOURCE_LEADER_REAL_POSITION = 2
uint8 SOURCE_SETPOINT = 3

uint8 source      # source classification for consumers that need real aircraft position

uint8 ORB_QUEUE_LENGTH = 8
