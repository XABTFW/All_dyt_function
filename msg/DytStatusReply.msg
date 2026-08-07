uint64 timestamp  # time since system start (microseconds)
uint64 timestamp_sample  # reply frame receive time (microseconds)

uint8 control_code
uint8 param_length
bool truncated

uint8[16] params

uint16 parse_error_count
