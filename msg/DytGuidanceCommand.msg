uint64 timestamp                 # time since system start (microseconds)
uint32 sequence                  # GCS request sequence used for acknowledgement
uint8 source_system
uint8 source_component
uint8 target_system              # 0 means broadcast
uint8 target_component           # 0 means autopilot component

uint8 PHASE_MIDCOURSE = 2
uint8 PHASE_TERMINAL = 3
uint8 phase

# ORB_QUEUE_LENGTH 4
