std_msgs/Header header

uint8 RECORDING_START=1
uint8 RECORDING_END=2
uint8 CONTACT_START=3
uint8 CONTACT_END=4
uint8 ABORT=5

uint8 kind
string session_id
string event_id
string region
string source
float32 target_force_n
float32 measured_force_n
float32 indentation_mm
uint32 mcu_uptime_ms
string notes
