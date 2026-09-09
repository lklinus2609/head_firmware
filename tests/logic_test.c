#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "control.h"
#include "lease.h"
#include "state_machine.h"

static float quiet_nan(void)
{
  const uint32_t bits = 0x7FC00000u;
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

static void make_valid_calibration(struct head_calibration *calibration)
{
  head_config_default(calibration);
  calibration->expected_servo_count = 1u;
  calibration->allow_partial_inventory = 1u;
  calibration->active_servo_mask = 1u;
  calibration->joints[0].homing_direction = 1.0f;
  calibration->joints[0].homing_max_travel_ticks = 100;
  calibration->joints[0].homing_timeout_ms = 1000u;
  calibration->joints[0].operating_current_ma = 100;
  calibration->joints[0].homing_current_ma = 50;
  calibration->joints[0].homing_following_error_ticks = 0;
  calibration->joints[0].homing_persistence_ms = 10u;
  calibration->joints[0].homing_current_limit_ma = 75;
  calibration->joints[0].homing_speed_ticks_per_second = 500u;
  calibration->joints[0].homing_backoff_ticks = 5;
  head_config_finalize(calibration);
  assert(head_config_validate(calibration));
}

static void test_non_finite_calibration_is_rejected(void)
{
  struct head_calibration calibration;
  make_valid_calibration(&calibration);

  calibration.joints[0].max_position_fraction_per_control_cycle = quiet_nan();
  head_config_finalize(&calibration);
  assert(!head_config_validate(&calibration));

  make_valid_calibration(&calibration);
  calibration.joints[0].max_acceleration_fraction_per_control_cycle_squared = quiet_nan();
  head_config_finalize(&calibration);
  assert(!head_config_validate(&calibration));

  make_valid_calibration(&calibration);
  calibration.joints[0].homing_direction = quiet_nan();
  head_config_finalize(&calibration);
  assert(!head_config_validate(&calibration));

  make_valid_calibration(&calibration);
  calibration.joints[0].reserved_homing_start_tick = 1;
  head_config_finalize(&calibration);
  assert(!head_config_validate(&calibration));
}

static void test_calibration_storage_schema_round_trip(void)
{
  struct head_calibration source;
  struct head_calibration decoded;
  uint8_t payload[HEAD_CONFIG_STORAGE_PAYLOAD_SIZE];
  make_valid_calibration(&source);
  source.joints[0].min_tick = -12345;
  source.joints[0].max_tick = 54321;
  source.joints[0].homing_direction = -1.0f;
  head_config_finalize(&source);

  assert(head_config_serialize(&source, payload, sizeof(payload)) ==
         HEAD_CONFIG_STORAGE_PAYLOAD_SIZE);
  assert(head_config_deserialize(payload, sizeof(payload), &decoded));
  assert(decoded.joints[0].min_tick == -12345);
  assert(decoded.joints[0].max_tick == 54321);
  assert(decoded.joints[0].homing_direction == -1.0f);
  assert(decoded.joints[0].homing_current_limit_ma == 75);
  assert(!head_config_deserialize(payload, sizeof(payload) - 1u, &decoded));

  payload[0] = 3u; /* Unsupported calibration schema version. */
  assert(!head_config_deserialize(payload, sizeof(payload), &decoded));
}

static void test_lease_expiry_and_rollover(void)
{
  struct head_runtime runtime = {0};
  runtime.active_lease_token = 7u;
  runtime.lease_expires_ms = 1100u;
  assert(head_lease_token_is_valid(&runtime, 7u, 1099u));
  assert(!head_lease_token_is_valid(&runtime, 7u, 1100u));
  assert(!head_lease_token_is_valid(&runtime, 8u, 1099u));

  head_lease_renew(&runtime, UINT32_MAX - 500u);
  assert(head_lease_token_is_valid(&runtime, 7u, UINT32_MAX - 1u));
  assert(head_lease_token_is_valid(&runtime, 7u, 200u));
  assert(!head_lease_token_is_valid(&runtime, 7u, 499u));
}

static void test_command_validation_and_sequence(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  struct head_command command = {0};
  make_valid_calibration(&calibration);

  runtime.state = HEAD_ENABLED;
  runtime.active_lease_token = 7u;
  runtime.lease_expires_ms = 2000u;
  command.lease_token = 7u;
  command.sequence = 1u;
  command.active_servo_mask = 1u;
  command.mode = HEAD_COMMAND_POSITION;
  command.position_normalized[0] = 0.5f;
  assert(head_control_accept_command(&runtime, &command, &calibration, 1000u));
  assert(runtime.last_accepted_sequence == 1u);
  assert(!head_control_accept_command(&runtime, &command, &calibration, 1001u));

  command.sequence = 2u;
  command.position_normalized[0] = quiet_nan();
  assert(!head_control_accept_command(&runtime, &command, &calibration, 1001u));

  command.position_normalized[0] = 0.5f;
  assert(!head_control_accept_command(&runtime, &command, &calibration, 2000u));
}

static void test_control_reinitialization_clears_runtime_state(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  runtime.homing_reference_valid[0] = true;
  runtime.branches[0].protocol_errors = 3u;
  runtime.accepted_sequence_valid = true;
  runtime.last_accepted_sequence = 9u;
  runtime.last_transmitted_sequence = 8u;

  head_control_init(&runtime, &calibration);

  assert(!runtime.homing_reference_valid[0]);
  assert(runtime.branches[0].protocol_errors == 0u);
  assert(!runtime.accepted_sequence_valid);
  assert(runtime.last_accepted_sequence == 0u);
  assert(runtime.last_transmitted_sequence == 0u);
}

static void test_proprioception_lifecycle(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  struct head_command command = {0};
  make_valid_calibration(&calibration);

  runtime.state = HEAD_READY;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  assert(!head_state_request_proprioception(&runtime, 1000u));

  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  runtime.accepted_sequence_valid = true;
  runtime.pending_transmit_valid = true;
  assert(head_state_request_proprioception(&runtime, 1000u));
  assert(runtime.state == HEAD_PROPRIOCEPTION_SETTLING);
  assert(!runtime.accepted_sequence_valid);
  assert(!runtime.pending_transmit_valid);

  command.lease_token = 7u;
  command.sequence = 1u;
  command.active_servo_mask = 1u;
  command.mode = HEAD_COMMAND_POSITION;
  command.position_normalized[0] = 0.5f;
  runtime.active_lease_token = 7u;
  runtime.lease_expires_ms = 5000u;
  assert(!head_control_accept_command(&runtime, &command, &calibration, 1100u));

  head_state_proprioception_tick(
      &runtime, 1000u + HEAD_PROPRIOCEPTION_SETTLE_MS - 1u);
  assert(runtime.state == HEAD_PROPRIOCEPTION_SETTLING);
  head_state_proprioception_tick(
      &runtime, 1000u + HEAD_PROPRIOCEPTION_SETTLE_MS);
  assert(runtime.state == HEAD_PROPRIOCEPTION_HOLD);

  head_state_disable(&runtime);
  assert(runtime.state == HEAD_HOMING_REQUIRED);
  assert(runtime.shutdown_requested);
  assert(runtime.torque_state == HEAD_TORQUE_SHUTDOWN_PENDING);
  assert(runtime.active_lease_token == 0u);
}

static void test_proprioception_settle_timer_rollover(void)
{
  struct head_runtime runtime = {0};
  runtime.state = HEAD_READY;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  const uint32_t started = UINT32_MAX - 1000u;
  assert(head_state_request_proprioception(&runtime, started));
  head_state_proprioception_tick(
      &runtime, started + HEAD_PROPRIOCEPTION_SETTLE_MS);
  assert(runtime.state == HEAD_PROPRIOCEPTION_HOLD);
}

static void test_routing_lifecycle_and_zero_hold(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  struct head_command command = {0};
  make_valid_calibration(&calibration);

  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  assert(!head_state_request_routing(&runtime, &calibration));

  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  runtime.servos[0].goal_tick = 2048;
  runtime.servos[0].goal_step_ticks_per_control_cycle = 0;
  runtime.active_lease_token = 7u;
  runtime.lease_expires_ms = 5000u;
  assert(head_state_request_routing(&runtime, &calibration));
  assert(runtime.state == HEAD_ROUTING);
  assert(HEAD_ROUTING == 10);

  command.lease_token = 7u;
  command.sequence = 1u;
  command.active_servo_mask = 1u;
  command.mode = HEAD_COMMAND_POSITION;
  command.position_normalized[0] = 0.5f;
  assert(!head_control_accept_command(&runtime, &command, &calibration, 1000u));

  for (uint32_t tick = 0u; tick < 10000u; ++tick) {
    head_control_tick(&runtime, &calibration, tick);
  }
  assert(runtime.servos[0].goal_tick == HEAD_ROUTING_BASE_TICK);
  assert(runtime.servos[0].goal_step_ticks_per_control_cycle == 0);

  head_state_disable(&runtime);
  assert(runtime.state == HEAD_HOMING_REQUIRED);
  assert(runtime.shutdown_requested);
  assert(runtime.active_lease_token == 0u);
}

static void test_routing_rejects_zero_outside_calibrated_range(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  calibration.joints[0].min_tick = 100;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  assert(!head_state_request_routing(&runtime, &calibration));
  assert(runtime.state == HEAD_HOMING_REQUIRED);
}

static void test_trajectory_reversal_is_bounded(void)
{
  const int32_t step = head_control_trajectory_step(3000, 20, 1000, 20, 1);
  assert(step == 19);
}

static void test_trajectory_invariants_and_convergence(void)
{
  int32_t position = 2048;
  int32_t step = 0;
  uint32_t state = 0x12345678u;

  for (uint32_t tick = 0u; tick < 200000u; ++tick) {
    state = state * 1664525u + 1013904223u;
    const int32_t target = tick < 190000u ? (int32_t)(state % 4096u) : 1024;
    const int32_t previous = step;
    step = head_control_trajectory_step(position, step, target, 20, 3);
    assert(step >= -20 && step <= 20);
    assert(step - previous >= -3 && step - previous <= 3);
    position += step;
    assert(position >= 0 && position <= 4095);
  }
  assert(position == 1024);
  assert(step == 0);
}

static void make_calibration_mask(struct head_calibration *calibration,
                                  uint32_t active_servo_mask)
{
  uint8_t active_count = 0u;
  head_config_default(calibration);
  calibration->allow_partial_inventory = 1u;
  calibration->active_servo_mask = active_servo_mask;
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((active_servo_mask & (1u << index)) == 0u) continue;
    struct head_joint_config *joint = &calibration->joints[index];
    joint->homing_direction = 1.0f;
    joint->homing_max_travel_ticks = 100;
    joint->homing_timeout_ms = 1000u;
    joint->operating_current_ma = 100;
    joint->homing_current_ma = 50;
    joint->homing_following_error_ticks = 0;
    joint->homing_persistence_ms = 10u;
    joint->homing_current_limit_ma = 75;
    joint->homing_speed_ticks_per_second = 500u;
    joint->homing_backoff_ticks = 5;
    ++active_count;
  }
  calibration->expected_servo_count = active_count;
  head_config_finalize(calibration);
  assert(head_config_validate(calibration));
}

static void run_one_homing_servo(struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t servo_index, uint32_t *now_ms)
{
  struct head_servo_state *servo = &runtime->servos[servo_index];
  servo->online = true;
  servo->present_voltage_mv = 8000u;
  servo->present_current_ma = calibration->joints[servo_index].homing_current_ma;
  servo->present_tick = 1000 + servo_index;
  servo->goal_tick = servo->present_tick;
  servo->last_feedback_ms = *now_ms;

  /* The first tick seeds the stage from fresh feedback. The next two ticks
   * qualify the stall for the configured persistence interval. */
  head_state_homing_tick(runtime, calibration, *now_ms);
  *now_ms += 2u;
  head_state_homing_tick(runtime, calibration, *now_ms);
  *now_ms += calibration->joints[servo_index].homing_persistence_ms;
  head_state_homing_tick(runtime, calibration, *now_ms);
  assert(runtime->homing_reference_valid[servo_index]);
  assert(runtime->homing_backoff_active);

  /* Feedback follows each backoff goal, as it would on the bus. */
  while (llabs((int64_t)servo->goal_tick -
               runtime->homing_zero_tick[servo_index]) <
         calibration->joints[servo_index].homing_backoff_ticks) {
    *now_ms += 2u;
    head_state_homing_tick(runtime, calibration, *now_ms);
    servo->present_tick = servo->goal_tick;
    servo->last_feedback_ms = *now_ms;
  }
  *now_ms += 2u;
  head_state_homing_tick(runtime, calibration, *now_ms);
}

static void test_terminal_homing_inventory(uint32_t active_servo_mask)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  uint32_t now_ms = 100u;
  make_calibration_mask(&calibration, active_servo_mask);
  head_state_init(&runtime);
  runtime.state = HEAD_HOMING;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  runtime.discovery_verified = true;
  runtime.homing_index = 0u;

  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((active_servo_mask & (1u << index)) == 0u) continue;
    assert(runtime.homing_index == index);
    run_one_homing_servo(&runtime, &calibration, index, &now_ms);
  }
  assert(runtime.state == HEAD_READY);
  assert(runtime.shutdown_requested);
  assert(runtime.torque_state == HEAD_TORQUE_SHUTDOWN_PENDING);
}

static void test_terminal_homing_single_sparse_and_full(void)
{
  test_terminal_homing_inventory(1u);
  test_terminal_homing_inventory((1u << 0u) | (1u << 7u) | (1u << 19u));
  test_terminal_homing_inventory((1u << HEAD_SERVO_COUNT) - 1u);
}

static void test_fault_latch_and_disable_recovery_gate(void)
{
  struct head_runtime runtime = {0};
  head_state_init(&runtime);
  runtime.state = HEAD_ENABLED;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  head_state_fault(&runtime, HEAD_FAULT_DISCOVERY);
  runtime.shutdown_next_attempt_ms = 5000u;
  head_state_disable(&runtime);

  assert(runtime.state == HEAD_FAULT);
  assert(runtime.fault == HEAD_FAULT_DISCOVERY);
  assert(runtime.shutdown_next_attempt_ms == 5000u);

  /* Even after a separately verified shutdown, an uncleared FAULT cannot
   * become HOMING_REQUIRED merely because Disable was requested. */
  runtime.shutdown_requested = false;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = true;
  struct head_calibration calibration;
  make_valid_calibration(&calibration);
  assert(!head_state_can_prepare_motion(&runtime));
  assert(!head_state_request_home(&runtime, &calibration));
}

static void test_fault_shutdown_is_idempotent(void)
{
  struct head_runtime runtime = {0};
  head_state_init(&runtime);
  runtime.state = HEAD_ENABLED;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  head_state_fault(&runtime, HEAD_FAULT_BUS);
  runtime.shutdown_next_attempt_ms = 7000u;
  runtime.shutdown_failures = 3u;
  runtime.shutdown_confirmations = 1u;

  head_state_fault(&runtime, HEAD_FAULT_WATCHDOG);
  head_state_disable(&runtime);
  assert(runtime.state == HEAD_FAULT);
  assert(runtime.fault == HEAD_FAULT_BUS);
  assert(runtime.shutdown_next_attempt_ms == 7000u);
  assert(runtime.shutdown_failures == 3u);
  assert(runtime.shutdown_confirmations == 1u);
}

static void test_storage_excludes_motion_preparation(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  head_state_init(&runtime);
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = true;

  const enum head_storage_state blocked_states[] = {
    HEAD_STORAGE_PENDING, HEAD_STORAGE_WRITING,
  };
  for (size_t index = 0u; index < sizeof(blocked_states) / sizeof(blocked_states[0]); ++index) {
    runtime.state = HEAD_HOMING_REQUIRED;
    runtime.storage_state = blocked_states[index];
    assert(!head_state_can_prepare_motion(&runtime));
    assert(!head_state_request_home(&runtime, &calibration));
    assert(!head_state_request_maintenance_calibration(&runtime, &calibration));
  }

  runtime.storage_state = HEAD_STORAGE_IDLE;
  runtime.preparation_active = true;
  assert(!head_state_can_prepare_motion(&runtime));
  runtime.preparation_active = false;
  runtime.diagnostic_active = true;
  assert(!head_state_can_prepare_motion(&runtime));
}

static void test_shifted_homing_reference_position_boundaries(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  calibration.joints[0].min_tick = 0;
  calibration.joints[0].max_tick = 4095;
  calibration.joints[0].home_tick = 2048;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  runtime.homing_reference_valid[0] = true;

  runtime.homing_zero_tick[0] = HEAD_DXL_POSITION_MAX_TICK;
  assert(!head_control_reference_valid(&runtime, &calibration, 0u));
  assert(head_control_target_tick(&runtime, &calibration, 0u) == 0);

  runtime.homing_zero_tick[0] = HEAD_DXL_POSITION_MAX_TICK -
                                (calibration.joints[0].max_tick -
                                 calibration.joints[0].home_tick);
  assert(head_control_reference_valid(&runtime, &calibration, 0u));
  runtime.requested_position_normalized[0] = 1.0f;
  assert(head_control_target_tick(&runtime, &calibration, 0u) ==
         HEAD_DXL_POSITION_MAX_TICK);

  runtime.homing_zero_tick[0] = HEAD_DXL_POSITION_MIN_TICK +
                                (calibration.joints[0].home_tick -
                                 calibration.joints[0].min_tick) - 1;
  assert(!head_control_reference_valid(&runtime, &calibration, 0u));
}

int main(void)
{
  test_non_finite_calibration_is_rejected();
  test_calibration_storage_schema_round_trip();
  test_lease_expiry_and_rollover();
  test_command_validation_and_sequence();
  test_control_reinitialization_clears_runtime_state();
  test_proprioception_lifecycle();
  test_proprioception_settle_timer_rollover();
  test_routing_lifecycle_and_zero_hold();
  test_routing_rejects_zero_outside_calibrated_range();
  test_trajectory_reversal_is_bounded();
  test_trajectory_invariants_and_convergence();
  test_terminal_homing_single_sparse_and_full();
  test_fault_latch_and_disable_recovery_gate();
  test_fault_shutdown_is_idempotent();
  test_storage_excludes_motion_preparation();
  test_shifted_homing_reference_position_boundaries();
  return 0;
}
