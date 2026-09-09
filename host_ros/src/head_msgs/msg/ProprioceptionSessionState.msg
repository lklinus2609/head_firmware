std_msgs/Header header

uint8 IDLE=0
uint8 ACQUIRING=1
uint8 HOMING=2
uint8 ENTERING_HOLD=3
uint8 WARMING_UP=4
uint8 ARMED=5
uint8 RECORDING=6
uint8 STOPPING=7
uint8 SAVED=8
uint8 ABORTING=9
uint8 ERROR=10

uint8 state
string session_id
string status
string output_path
bool can_start_recording
bool can_stop_recording
uint64 samples_received
uint64 samples_invalid
uint32 warmup_remaining_ms
