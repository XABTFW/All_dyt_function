uint64 timestamp                 # time since system start (microseconds)
uint32 command_sequence          # latest GCS command observed by this module

uint8 ROLE_BROADCAST = 1
uint8 ROLE_RENDEZVOUS = 2
uint8 role

bool enabled                     # RC or GCS has enabled cooperative guidance
bool active                      # module is actively providing midcourse behavior
bool controlling_vehicle         # module currently owns trajectory setpoints
bool target_valid                # fresh cooperative target position is available
