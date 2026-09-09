std_msgs/Header header
# Firmware lifecycle-state values. Existing values are append-only on the
# protocol so recorded datasets remain interpretable across releases.
uint8 BOOT=0
uint8 HOMING_REQUIRED=2
uint8 HOMING=3
uint8 MAINTENANCE_CALIBRATION=4
uint8 READY=5
uint8 ENABLED=6
uint8 FAULT=7
uint8 PROPRIOCEPTION_SETTLING=8
uint8 PROPRIOCEPTION_HOLD=9
uint8 ROUTING=10
uint8 state
uint8 fault
bool torque_enabled
uint8 torque_state
bool shutdown_pending
uint8 shutdown_confirmations
uint32 shutdown_attempts
uint32 shutdown_failures
bool fan_stalled
uint16 fan_rpm
uint32 mcu_uptime_ms
# Protocol-v2 compatibility name: last sequence transmitted to all branches,
# not a Status-Packet-confirmed physical application.
uint32 applied_sequence
uint16 control_hz
uint32 boot_session_id
uint32 active_servo_mask
head_msgs/ServoState[] servos
