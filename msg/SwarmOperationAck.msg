uint64 timestamp          # time since system start (microseconds)

uint8 target_system       # target system ID (aircraft that performed the operation)
uint8 operation_type      # operation type
uint8 result              # operation result
uint32 old_value          # previous value before operation
uint32 new_value          # new value after operation

# Operation type constants
uint8 OP_GROUP_CHANGE = 1
uint8 OP_LEADER_CHANGE = 2
uint8 OP_TAKEOFF = 3
uint8 OP_LAND = 4
uint8 OP_PAUSE = 5
uint8 OP_CONTINUE = 6

# Result constants
uint8 RESULT_SUCCESS = 0
uint8 RESULT_FAILED = 1
