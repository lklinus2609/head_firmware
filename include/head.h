#ifndef HEAD_H_
#define HEAD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HEAD_SERVO_COUNT 20u
#define HEAD_BRANCH_COUNT 4u
#define HEAD_SERVOS_PER_BRANCH 5u
#define HEAD_CONTROL_HZ 500u
#define HEAD_CONTROL_PERIOD_US (1000000u / HEAD_CONTROL_HZ)
#define HEAD_CONTROL_FATAL_DELAY_US 5000u
#define HEAD_TELEMETRY_HZ 100u
#define HEAD_COMMAND_TIMEOUT_MS 100u
#define HEAD_TIMEOUT_HOLD_MS 50u
#define HEAD_LEASE_DURATION_MS 1000u
#define HEAD_TELEMETRY_PERIOD_MS 10u
/* Five 88-byte replies require 4.4 ms at the current 1 Mbps setting,
 * before byte stuffing and UART idle-delivery scheduling. */
/* Admission window for a motion command: feedback this old or newer. */
#define HEAD_COMMAND_FEEDBACK_FRESH_MS 30u
#define HEAD_TELEMETRY_RESPONSE_TIMEOUT_MS 8u
/* Homing acts on measured position every cycle, so it must notice feedback
 * that has stopped -- but it must tolerate a lost Sync Read. One miss costs a
 * 10 ms poll plus its 8 ms timeout before the next attempt even starts, so
 * this allows two consecutive misses and still faults on a real stall. */
/* Arrival window for zero homing: ~1.8 degrees on a 4096-tick revolution. */
#define HEAD_ZERO_HOME_SETTLE_TICKS 20
#define HEAD_HOMING_FEEDBACK_STALE_MS 60u
/* Supervisor limit while torque may be on: holding torque on feedback that
 * has stopped is the hazard, so this must stay short -- but it has to outlast
 * one lost Sync Read (a 10 ms poll plus its 8 ms timeout) or it fires on a
 * healthy bus. Same budget as the homing window, for the same reason. */
#define HEAD_TELEMETRY_STALE_FAULT_MS 60u
#define HEAD_PROPRIOCEPTION_SETTLE_MS 2000u
/* Each physical servo branch is protected by a 3 A fuse. The measured
 * servo-current sum of a branch above this faults the head. Configured totals
 * are not limited (decided 2026-10-06). */
#define HEAD_BRANCH_CURRENT_BUDGET_MA 2500u
#define HEAD_ROUTING_BASE_TICK 0
/* One revolution of the servo's absolute encoder. Multi-turn Present Position
 * is this many ticks per turn, so the encoder zeros are its multiples. */
#define HEAD_ENCODER_TICKS_PER_REVOLUTION 4096
/* Largest single tuning step. A step away from neutral is refused while the
 * servo still trails its goal, so steps cannot pile up into a larger jump. */
#define HEAD_TUNING_MAX_STEP_TICKS 50
/* A tendon's full travel never exceeds one turn in intended use, so tuning
 * refuses any goal farther than one revolution from the pose it started at. */
#define HEAD_TUNING_MAX_TRAVEL_TICKS HEAD_ENCODER_TICKS_PER_REVOLUTION
/* A step away from neutral is refused once the servo trails its goal by
 * the tuning current limit divided by HEAD_TUNING_LAG_CURRENT_MA_PER_TICK,
 * and never less than HEAD_TUNING_MIN_LAG_TICKS. With Position I = 0 (set at
 * discovery) the servo's current is proportional to that lag, so this is the
 * lag at which it is already pulling with the limit the operator typed: the
 * limit, not a fixed window, decides where a step stops. It also bounds how
 * far a blocked servo's goal can lead, and so how far it can jump if the
 * mechanism gives way. The ratio was 3.08 to 3.76 mA per tick over 68
 * refused steps on servos 1, 7, 9, 17 and 18 (tuning record of 2026-10-06,
 * Position P 900); 3 is below all of them, so the lag limit is reached no
 * earlier than the current limit. */
#define HEAD_TUNING_LAG_CURRENT_MA_PER_TICK 3u
/* Floor for low limits: holding noise and gearbox lash alone need this much.
 * Same window zero homing uses to declare arrival. */
#define HEAD_TUNING_MIN_LAG_TICKS HEAD_ZERO_HOME_SETTLE_TICKS
#define HEAD_DXL_POSITION_MIN_TICK (-1048575)
#define HEAD_DXL_POSITION_MAX_TICK 1048575
/* Co-actuated modiolus groups (tuning_runs/servo_notes.json; new head
 * 2026-11-09): right 8 down / 11 diag / 14 up, left 9 down / 15 diag / 19 up. */
#define HEAD_COACTUATED_GROUP_A_MASK ((1u << 8u) | (1u << 11u) | (1u << 14u))
#define HEAD_COACTUATED_GROUP_B_MASK ((1u << 9u) | (1u << 15u) | (1u << 19u))

enum head_command_mode {
  HEAD_COMMAND_POSITION = 0,
  HEAD_COMMAND_POSITION_VELOCITY = 1,
  HEAD_COMMAND_VELOCITY = 2,
};

enum head_state {
  HEAD_BOOT = 0,
  HEAD_HOMING_REQUIRED = 2,
  HEAD_HOMING,
  HEAD_MAINTENANCE_CALIBRATION,
  HEAD_READY,
  HEAD_ENABLED,
  HEAD_FAULT,
  /* Append-only protocol values: preserve the established v2 state numbers. */
  HEAD_PROPRIOCEPTION_SETTLING,
  HEAD_PROPRIOCEPTION_HOLD,
  /* Fixture-wiring hold at the Dynamixel 0-degree encoder position. */
  HEAD_ROUTING,
  /* Commissioning: every active servo holds where it was at torque-on, and
   * the operator jogs one servo at a time in single-tick steps. */
  HEAD_TUNING,
};

enum head_fault {
  HEAD_FAULT_NONE = 0,
  HEAD_FAULT_CONFIGURATION,
  HEAD_FAULT_DISCOVERY,
  HEAD_FAULT_HOMING,
  HEAD_FAULT_WATCHDOG,
  HEAD_FAULT_BUS,
  HEAD_FAULT_SERVO,
  HEAD_FAULT_FAN,
  HEAD_FAULT_CONTROL_DEADLINE,
  HEAD_FAULT_TELEMETRY,
  /* Reserved legacy protocol value. The former whole-head budget has been
   * removed and current firmware never emits this fault. */
  HEAD_FAULT_CURRENT_BUDGET,
  /* Append-only protocol value: one physical branch exceeded its commissioned
   * aggregate current budget. */
  HEAD_FAULT_BRANCH_CURRENT_BUDGET,
};

enum head_torque_state {
  HEAD_TORQUE_UNKNOWN = 0,
  HEAD_TORQUE_OFF_VERIFIED,
  HEAD_TORQUE_ON_VERIFIED,
  HEAD_TORQUE_SHUTDOWN_PENDING,
  HEAD_TORQUE_SHUTDOWN_FAILED,
};

enum head_storage_state {
  HEAD_STORAGE_IDLE = 0,
  HEAD_STORAGE_PENDING,
  HEAD_STORAGE_WRITING,
  HEAD_STORAGE_SUCCEEDED,
  HEAD_STORAGE_FAILED,
};

struct head_joint_config {
  /* Serialized v4/JSON names remain branch and id for compatibility. */
  uint8_t branch_index;
  uint8_t servo_id;
  int32_t min_tick;
  int32_t max_tick;
  int32_t home_tick;
  float max_position_fraction_per_control_cycle;
  /* v4 JSON alias: max_acceleration_fraction_per_control_cycle. */
  float max_acceleration_fraction_per_control_cycle_squared;
  float homing_direction;
  /* Reserved in calibration schema v4; must be zero. */
  int32_t reserved_homing_start_tick;
  int32_t homing_max_travel_ticks;
  uint32_t homing_timeout_ms;
  int16_t operating_current_ma;
  int16_t homing_current_ma;
  int32_t homing_following_error_ticks;
  uint16_t homing_persistence_ms;
  int16_t homing_current_limit_ma;
  uint16_t homing_speed_ticks_per_second;
  int32_t homing_backoff_ticks;
};

struct head_calibration {
  uint32_t version;
  uint8_t expected_servo_count;
  uint8_t allow_partial_inventory;
  uint8_t thermal_start_c;
  uint8_t thermal_full_c;
  uint32_t active_servo_mask;
  struct head_joint_config joints[HEAD_SERVO_COUNT];
  uint32_t crc32;
};

struct head_servo_state {
  int32_t goal_tick;
  int32_t present_tick;
  /* Signed Dynamixel Present Velocity register value (unit: 0.229 rpm). */
  int32_t present_velocity_raw;
  int16_t present_current_ma;
  /* Signed Present PWM(124) register value (unit: 0.113 %). */
  int16_t present_pwm_raw;
  uint16_t present_voltage_mv;
  uint8_t temperature_c;
  uint8_t moving_status;
  uint8_t hardware_error;
  uint8_t firmware_version;
  bool online;
  uint32_t last_feedback_ms;
  /* Updated only by full telemetry that includes Present Current(126), never
   * by position-only preparation reads. */
  uint32_t last_current_feedback_ms;
  int32_t goal_step_ticks_per_control_cycle;
};

struct head_branch_health {
  uint32_t telemetry_requested_ms;
  uint32_t telemetry_completed_ms;
  uint32_t telemetry_timeouts;
  uint32_t protocol_errors;
  uint32_t bus_errors;
  uint32_t control_transmissions;
  uint32_t control_transmission_errors;
  uint32_t control_transmission_deferred;
  bool telemetry_active;
  uint8_t telemetry_expected_mask;
  uint8_t telemetry_received_mask;
};

struct head_command {
  uint32_t lease_token;
  uint32_t sequence;
  uint8_t mode;
  float position_normalized[HEAD_SERVO_COUNT];
  float velocity_normalized[HEAD_SERVO_COUNT];
  uint32_t active_servo_mask;
};

struct head_runtime {
  enum head_state state;
  enum head_fault fault;
  uint32_t active_lease_token;
  uint32_t lease_expires_ms;
  uint32_t last_command_ms;
  uint32_t watchdog_hold_started_ms;
  uint32_t last_control_cycle_ms;
  uint32_t control_deadline_misses;
  uint32_t control_max_period_us;
  uint32_t shutdown_attempts;
  uint32_t shutdown_failures;
  uint32_t shutdown_next_attempt_ms;
  uint8_t shutdown_confirmations;
  uint32_t last_accepted_sequence;
  uint32_t last_transmitted_sequence;
  uint32_t pending_transmit_sequence;
  uint8_t pending_transmit_branch_mask;
  uint32_t homing_started_ms;
  uint32_t homing_qualified_since_ms;
  uint32_t proprioception_started_ms;
  uint8_t homing_index;
  uint8_t homing_torque_index;
  /* Homing seeks the encoder's absolute zero instead of a mechanical stop. */
  bool zero_homing;
  bool maintenance_calibration;
  bool maintenance_waiting_confirm;
  bool homing_backoff_active;
  enum head_torque_state torque_state;
  bool shutdown_requested;
  bool discovery_verified;
  bool discovery_active;
  bool preparation_active;
  uint8_t preparation_phase;
  uint8_t preparation_servo_index;
  /* Consecutive transient bus failures on the current preparation step. */
  uint8_t preparation_attempts;
  enum head_state preparation_target_state;
  uint32_t preparation_started_ms;
  uint32_t preparation_completed_ms;
  bool diagnostic_active;
  bool diagnostic_cancel_requested;
  bool accepted_sequence_valid;
  bool pending_transmit_valid;
  bool fan_override;
  uint8_t fan_override_percent;
  enum head_storage_state storage_state;
  int32_t storage_result;
  int32_t homing_zero_tick[HEAD_SERVO_COUNT];
  int32_t homing_origin_tick[HEAD_SERVO_COUNT];
  uint32_t homing_motion_remainder_ticks_per_second[HEAD_SERVO_COUNT];
  bool homing_reference_valid[HEAD_SERVO_COUNT];
  float requested_position_normalized[HEAD_SERVO_COUNT];
  /* Goal Current(102) for every active servo during tuning, and each servo's
   * goal when tuning began; travel is bounded relative to that neutral pose. */
  uint16_t tuning_current_limit_ma;
  int32_t tuning_neutral_tick[HEAD_SERVO_COUNT];
  /* Changes whenever the tick that q = 0 or q = 1 maps to may have changed
   * (profile applied, homing reference set or cleared). STATE carries it so a
   * host can tell that its copy of POSITION_RANGE is out of date. */
  uint8_t position_range_generation;
  struct head_servo_state servos[HEAD_SERVO_COUNT];
  struct head_branch_health branches[HEAD_BRANCH_COUNT];
};

#endif
