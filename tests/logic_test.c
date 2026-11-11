#include <errno.h>
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

static void test_branch_current_budget(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  calibration.expected_servo_count = 3u;
  calibration.active_servo_mask = 7u;
  calibration.joints[0].operating_current_ma = 833;
  calibration.joints[1] = calibration.joints[0];
  calibration.joints[1].servo_id = 1u;
  calibration.joints[2] = calibration.joints[0];
  calibration.joints[2].servo_id = 2u;
  calibration.joints[2].operating_current_ma = 834;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));

  /* Configured branch totals are not limited (decided 2026-10-06); the
   * measured branch current is supervised live instead. */
  calibration.joints[2].operating_current_ma = 835;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));

  runtime.servos[0].present_current_ma = -900;
  runtime.servos[1].present_current_ma = 901;
  runtime.servos[2].present_current_ma = 700;
  assert(head_control_present_current_branch_ma(&runtime, &calibration, 0u) == 2501u);

  make_valid_calibration(&calibration);
  calibration.expected_servo_count = 3u;
  calibration.active_servo_mask = (1u << 0u) | (1u << 5u) | (1u << 10u);
  for (uint8_t index = 5u; index <= 10u; index += 5u) {
    calibration.joints[index] = calibration.joints[0];
    calibration.joints[index].branch_index = index / HEAD_SERVOS_PER_BRANCH;
    calibration.joints[index].servo_id = index;
  }
  calibration.joints[0].operating_current_ma = 900;
  calibration.joints[5].operating_current_ma = 900;
  calibration.joints[10].operating_current_ma = 900;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
}

static void test_profile_declares_branch(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  uint8_t servo_indexes[HEAD_SERVOS_PER_BRANCH];

  /* IDs 15-19 on J2 and 5-9 on J4, the cabling found on 2026-09-28. */
  make_valid_calibration(&calibration);
  calibration.expected_servo_count = 10u;
  calibration.active_servo_mask = 0u;
  for (uint8_t index = 5u; index < 10u; ++index) {
    calibration.joints[index] = calibration.joints[0];
    calibration.joints[index].servo_id = index;
    calibration.joints[index].branch_index = 3u;
    calibration.joints[index + 10u] = calibration.joints[0];
    calibration.joints[index + 10u].servo_id = (uint8_t)(index + 10u);
    calibration.joints[index + 10u].branch_index = 1u;
    calibration.active_servo_mask |= (1u << index) | (1u << (index + 10u));
  }
  calibration.joints[7].operating_current_ma = 300;
  calibration.joints[17].operating_current_ma = 200;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  assert(head_config_branch_servo_indexes(&calibration, 1u, servo_indexes) == 5u);
  assert(servo_indexes[0] == 15u && servo_indexes[4] == 19u);
  assert(head_config_branch_servo_indexes(&calibration, 3u, servo_indexes) == 5u);
  assert(servo_indexes[0] == 5u && servo_indexes[4] == 9u);
  assert(head_config_branch_servo_indexes(&calibration, 0u, servo_indexes) == 0u);
  assert(head_config_branch_servo_indexes(&calibration, 2u, servo_indexes) == 0u);

  memset(&runtime, 0, sizeof(runtime));
  runtime.servos[7].present_current_ma = 1000;
  runtime.servos[17].present_current_ma = -400;
  assert(head_control_present_current_branch_ma(&runtime, &calibration, 3u) == 1000u);
  assert(head_control_present_current_branch_ma(&runtime, &calibration, 1u) == 400u);

  /* Five servos at 600 mA on one branch (3,000 mA configured) are accepted. */
  for (uint8_t index = 5u; index < 10u; ++index) {
    calibration.joints[index].operating_current_ma = 600;
  }
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));

  /* A sixth active servo on one branch exceeds the Sync Read window. */
  make_valid_calibration(&calibration);
  calibration.expected_servo_count = 6u;
  calibration.active_servo_mask = 0u;
  for (uint8_t index = 0u; index < 6u; ++index) {
    calibration.joints[index * 3u] = calibration.joints[0];
    calibration.joints[index * 3u].servo_id = (uint8_t)(index * 3u);
    calibration.joints[index * 3u].branch_index = 2u;
    calibration.active_servo_mask |= 1u << (index * 3u);
  }
  head_config_finalize(&calibration);
  assert(!head_config_validate(&calibration));
  assert(head_config_branch_servo_indexes(&calibration, 2u, servo_indexes) ==
         HEAD_SERVOS_PER_BRANCH);
  calibration.joints[15].branch_index = 0u;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));

  /* An active servo must name a real branch; an inactive one is ignored. */
  make_valid_calibration(&calibration);
  calibration.joints[0].branch_index = HEAD_BRANCH_COUNT;
  head_config_finalize(&calibration);
  assert(!head_config_validate(&calibration));
  make_valid_calibration(&calibration);
  calibration.joints[4].branch_index = 200u;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
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
    /* A full branch remains well within its 2,500 mA budget while sparse
     * fixtures exercise the same lifecycle paths. */
    joint->operating_current_ma = 50;
    joint->homing_current_ma = 25;
    joint->homing_following_error_ticks = 0;
    joint->homing_persistence_ms = 10u;
    joint->homing_current_limit_ma = 40;
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

/* Zero homing has no stop to find: it drives to the nearest encoder zero and
 * takes that as the datum, so the reference must land on an exact multiple of a
 * revolution regardless of which side the joint started on. */
static void run_one_zero_homing_servo(struct head_runtime *runtime,
                                      const struct head_calibration *calibration,
                                      uint8_t servo_index, int32_t start_tick,
                                      int32_t expected_zero, uint32_t *now_ms)
{
  struct head_servo_state *servo = &runtime->servos[servo_index];
  servo->online = true;
  servo->present_voltage_mv = 8000u;
  servo->present_current_ma = 0;
  servo->present_tick = start_tick;
  servo->goal_tick = start_tick;
  servo->last_feedback_ms = *now_ms;

  head_state_homing_tick(runtime, calibration, *now_ms);
  /* Feedback follows the commanded goal, as it would on the bus. */
  for (uint32_t tick = 0u; tick < 20000u &&
       !runtime->homing_reference_valid[servo_index]; ++tick) {
    *now_ms += 1u;
    servo->present_tick = servo->goal_tick;
    servo->last_feedback_ms = *now_ms;
    head_state_homing_tick(runtime, calibration, *now_ms);
    if (runtime->fault != HEAD_FAULT_NONE) break;
  }
  assert(runtime->fault == HEAD_FAULT_NONE);
  assert(runtime->homing_reference_valid[servo_index]);
  assert(runtime->homing_zero_tick[servo_index] == expected_zero);
  /* The encoder's own zero, the expressible one of the two 0/360 ticks. */
  assert(runtime->homing_zero_tick[servo_index] == 0);
}

static void test_zero_homing_reaches_encoder_zero(void)
{
  const struct {
    int32_t start;
    int32_t expected;
  } cases[] = {
    /* Every revolution is the same shaft angle, but only tick 0 is inside the
     * servo's Max Position Limit, so that is the datum from anywhere. */
    { 690, 0 },
    { -923, 0 },
    { 3000, 0 },
    { -3000, 0 },
    { 0, 0 },            /* already there: no motion needed */
  };
  for (size_t index = 0u; index < sizeof(cases) / sizeof(cases[0]); ++index) {
    struct head_calibration calibration;
    struct head_runtime runtime = {0};
    uint32_t now_ms = 100u;
    make_calibration_mask(&calibration, 1u);
    /* A full half-revolution of travel, as the bench profile now allows. */
    /* Two revolutions, as the bench profile allows: reaching a mid-revolution
     * datum from a multi-turn position can need more than one turn. */
    calibration.joints[0].homing_max_travel_ticks = 8192;
    calibration.joints[0].homing_timeout_ms = 60000u;
    head_config_finalize(&calibration);
    assert(head_config_validate(&calibration));
    head_state_init(&runtime);
    /* Requesting motion needs a verified-off bus; preparation is what turns
     * torque on, so mirror that ordering rather than starting energized. */
    runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
    runtime.discovery_verified = true;
    runtime.state = HEAD_HOMING_REQUIRED;
    assert(head_state_request_zero_home(&runtime, &calibration));
    assert(runtime.zero_homing);
    assert(runtime.state == HEAD_HOMING);
    runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
    run_one_zero_homing_servo(&runtime, &calibration, 0u, cases[index].start,
                              cases[index].expected, &now_ms);
    /* Arrive and hold: an encoder-zero datum has no mechanical stop holding
     * the joint, so releasing torque on arrival would let it drift straight
     * back off the position just established. */
    assert(runtime.state == HEAD_PROPRIOCEPTION_HOLD);
    assert(!runtime.shutdown_requested);
    assert(runtime.torque_state == HEAD_TORQUE_ON_VERIFIED);
    /* The datum must not survive a disable: the turn count is only valid for
     * as long as the servo stays powered. */
    head_state_disable(&runtime);
    assert(!runtime.zero_homing);
    assert(runtime.shutdown_requested);
  }
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

/* The diagnostic escape hatch for a silent bus must open only when nothing can
 * be energized, and must stay shut for every servo that was ever torque-on. */
static void test_torque_never_commanded_on_gate(void)
{
  struct head_runtime runtime = {0};
  head_state_init(&runtime);

  /* Boot: not a fault, so no relaxation regardless of torque state. */
  assert(!head_state_torque_never_commanded_on(&runtime));

  /* Discovery failed before any torque-on. The shutdown sweep parks here
   * forever against silent servos, and read-only diagnostics stay reachable. */
  head_state_fault(&runtime, HEAD_FAULT_DISCOVERY);
  assert(runtime.torque_state == HEAD_TORQUE_SHUTDOWN_PENDING);
  assert(head_state_torque_may_be_on(&runtime));
  assert(head_state_torque_never_commanded_on(&runtime));
  runtime.torque_state = HEAD_TORQUE_SHUTDOWN_FAILED;
  assert(head_state_torque_never_commanded_on(&runtime));

  /* A servo that reached torque-on is never covered, in any later state: the
   * shutdown states erase ON_VERIFIED, so discovery_verified carries the fact
   * that preparation once ran. */
  struct head_runtime energized = {0};
  head_state_init(&energized);
  energized.discovery_verified = true;
  energized.state = HEAD_ENABLED;
  energized.torque_state = HEAD_TORQUE_ON_VERIFIED;
  assert(!head_state_torque_never_commanded_on(&energized));
  head_state_fault(&energized, HEAD_FAULT_BUS);
  assert(energized.torque_state == HEAD_TORQUE_SHUTDOWN_PENDING);
  assert(!head_state_torque_never_commanded_on(&energized));
  energized.torque_state = HEAD_TORQUE_SHUTDOWN_FAILED;
  assert(!head_state_torque_never_commanded_on(&energized));

  /* ON_VERIFIED is refused even if discovery_verified were somehow clear. */
  struct head_runtime on_verified = {0};
  head_state_init(&on_verified);
  on_verified.state = HEAD_FAULT;
  on_verified.torque_state = HEAD_TORQUE_ON_VERIFIED;
  assert(!head_state_torque_never_commanded_on(&on_verified));

  /* The relaxation must not become a motion path. */
  struct head_calibration calibration;
  make_valid_calibration(&calibration);
  assert(!head_state_can_prepare_motion(&runtime));
  assert(!head_state_request_home(&runtime, &calibration));
  assert(!head_state_request_enable(&runtime));
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

/* 2026-10-20: one joint (the jaw) may be allowed more than the others. Each
 * servo's tuning cap is the requested limit or its own operating_current_ma,
 * whichever is lower, for the Goal Current and for refusing steps. */
static void test_tuning_per_servo_limit(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);  /* servo 0 active, 100 mA operating */
  /* Servo 1 set up like servo 0 (its default branch and ID), but 650 mA. */
  calibration.expected_servo_count = 2u;
  calibration.active_servo_mask = 0x3u;
  calibration.joints[1].homing_direction = 1.0f;
  calibration.joints[1].homing_max_travel_ticks = 100;
  calibration.joints[1].homing_timeout_ms = 1000u;
  calibration.joints[1].operating_current_ma = 650;
  calibration.joints[1].homing_current_ma = 50;
  calibration.joints[1].homing_following_error_ticks = 0;
  calibration.joints[1].homing_persistence_ms = 10u;
  calibration.joints[1].homing_current_limit_ma = 75;
  calibration.joints[1].homing_speed_ticks_per_second = 500u;
  calibration.joints[1].homing_backoff_ticks = 5;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  head_state_init(&runtime);
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = true;

  /* Above the highest operating current caps nothing: refused. */
  assert(!head_state_request_tuning(&runtime, &calibration, 651u));
  assert(head_state_request_tuning(&runtime, &calibration, 650u));
  assert(head_state_tuning_limit_ma(&runtime, &calibration, 0u) == 100u);
  assert(head_state_tuning_limit_ma(&runtime, &calibration, 1u) == 650u);
  runtime.tuning_current_limit_ma = 300u;
  assert(head_state_tuning_limit_ma(&runtime, &calibration, 0u) == 100u);
  assert(head_state_tuning_limit_ma(&runtime, &calibration, 1u) == 300u);
  runtime.tuning_current_limit_ma = 650u;

  /* Both servos read 120 mA: servo 0 is past its own 100 mA cap, servo 1 is
   * well inside its 650 mA one. */
  runtime.state = HEAD_TUNING;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  for (uint8_t index = 0u; index < 2u; ++index) {
    runtime.servos[index].goal_tick = 1000;
    runtime.servos[index].present_tick = 1000;
    runtime.tuning_neutral_tick[index] = 1000;
    runtime.servos[index].last_current_feedback_ms = 500u;
    runtime.servos[index].present_current_ma = 120;
  }
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EBUSY);
  assert(head_state_tuning_step(&runtime, &calibration, 1u, 1, 510u) == 0);
  /* The lag limit follows each servo's own cap too: 100 / 3 = 33 ticks is
   * past servo 0's limit but inside servo 1's 650 / 3 = 216. */
  runtime.servos[0].present_current_ma = 0;
  runtime.servos[1].present_current_ma = 0;
  runtime.servos[0].goal_tick = 1040;
  runtime.servos[1].goal_tick = 1040;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EBUSY);
  assert(head_state_tuning_step(&runtime, &calibration, 1u, 1, 510u) == 0);
}

static void test_tuning_request_and_steps(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);  /* servo 0 active, 100 mA operating */
  head_state_init(&runtime);
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = true;

  /* The limit must be set and may not exceed operating_current_ma. */
  assert(!head_state_request_tuning(&runtime, &calibration, 0u));
  assert(!head_state_request_tuning(&runtime, &calibration, 101u));
  assert(head_state_request_tuning(&runtime, &calibration, 40u));
  assert(runtime.tuning_current_limit_ma == 40u);
  assert(runtime.state == HEAD_HOMING_REQUIRED);  /* preparation changes state */
  runtime.state = HEAD_READY;
  assert(!head_state_request_tuning(&runtime, &calibration, 40u));

  /* Preparation finished into TUNING at a pose 1000 ticks from zero. */
  runtime.state = HEAD_TUNING;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  runtime.servos[0].goal_tick = 1000;
  runtime.servos[0].present_tick = 1000;
  runtime.tuning_neutral_tick[0] = 1000;
  runtime.servos[0].last_current_feedback_ms = 500u;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == 0);
  assert(runtime.servos[0].goal_tick == 1001);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == 0);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == 0);
  assert(runtime.servos[0].goal_tick == 999);

  /* A large step moves the whole amount at once. */
  assert(head_state_tuning_step(&runtime, &calibration, 0u, HEAD_TUNING_MAX_STEP_TICKS, 510u) == 0);
  assert(runtime.servos[0].goal_tick == 999 + HEAD_TUNING_MAX_STEP_TICKS);
  /* The servo has not caught up yet, so the next step away is refused... */
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 10, 510u) == -EBUSY);
  /* ...but stepping back is allowed. */
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -HEAD_TUNING_MAX_STEP_TICKS, 510u) == 0);
  assert(runtime.servos[0].goal_tick == 999);

  /* Bad requests and stale feedback move nothing. */
  assert(head_state_tuning_step(&runtime, &calibration, 1u, 1, 510u) == -EINVAL);
  assert(head_state_tuning_step(&runtime, &calibration, 0u,
                                (int8_t)(HEAD_TUNING_MAX_STEP_TICKS + 1), 510u) == -EINVAL);
  assert(head_state_tuning_step(&runtime, &calibration, 0u,
                                (int8_t)-(HEAD_TUNING_MAX_STEP_TICKS + 1), 510u) == -EINVAL);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 0, 510u) == -EINVAL);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1,
                                500u + HEAD_COMMAND_FEEDBACK_FRESH_MS + 1u) == -EAGAIN);
  assert(runtime.servos[0].goal_tick == 999);

  /* Saturated at the limit: away from neutral is refused, back is allowed. */
  runtime.servos[0].present_current_ma = -40;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == -EBUSY);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == 0);
  assert(runtime.servos[0].goal_tick == 1000);
  runtime.servos[0].present_current_ma = 39;

  /* Trailing the goal means the mechanism resists: same rule. */
  runtime.servos[0].goal_tick = 1100;
  runtime.servos[0].present_tick = 1100 - HEAD_TUNING_MIN_LAG_TICKS;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EBUSY);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == 0);
  runtime.servos[0].present_tick = runtime.servos[0].goal_tick;

  /* One turn from neutral is the end of travel, in both directions. */
  runtime.servos[0].goal_tick = 1000 + HEAD_TUNING_MAX_TRAVEL_TICKS;
  runtime.servos[0].present_tick = runtime.servos[0].goal_tick;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -ERANGE);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == 0);
  runtime.servos[0].goal_tick = 1000 - HEAD_TUNING_MAX_TRAVEL_TICKS;
  runtime.servos[0].present_tick = runtime.servos[0].goal_tick;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == -ERANGE);

  /* Outside TUNING, or after a fault, no step is accepted. */
  head_state_fault(&runtime, HEAD_FAULT_BUS);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EPERM);
  runtime = (struct head_runtime){0};
  runtime.state = HEAD_ENABLED;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  runtime.servos[0].last_current_feedback_ms = 500u;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EPERM);
}

/* Seeding must reproduce the exact goal tick, or Enable moves the servo. */
static void check_seed_round_trip(int32_t min_tick, int32_t max_tick, int32_t stride)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  calibration.joints[0].min_tick = min_tick;
  calibration.joints[0].max_tick = max_tick;
  calibration.joints[0].home_tick = min_tick + (max_tick - min_tick) / 2;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  for (int64_t goal = min_tick; goal <= max_tick; goal += stride) {
    runtime.servos[0].goal_tick = (int32_t)goal;
    assert(head_control_seed_requested_positions(&runtime, &calibration));
    assert(head_control_target_tick(&runtime, &calibration, 0u) == (int32_t)goal);
  }
  runtime.servos[0].goal_tick = max_tick;
  assert(head_control_seed_requested_positions(&runtime, &calibration));
  assert(head_control_target_tick(&runtime, &calibration, 0u) == max_tick);
  runtime.servos[0].goal_tick = min_tick - 1;
  assert(!head_control_seed_requested_positions(&runtime, &calibration));
  runtime.servos[0].goal_tick = max_tick + 1;
  assert(!head_control_seed_requested_positions(&runtime, &calibration));
}

static void test_enable_seeds_requested_positions_exactly(void)
{
  check_seed_round_trip(0, 4095, 1);
  check_seed_round_trip(-5000, -1012, 1);
  check_seed_round_trip(-4096, 4096, 1);
  check_seed_round_trip(HEAD_DXL_POSITION_MIN_TICK, HEAD_DXL_POSITION_MAX_TICK, 7);
}

static void test_enable_at_present_pose_request(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  int32_t minimum = 0;
  int32_t maximum = 0;
  make_valid_calibration(&calibration);
  calibration.joints[0].min_tick = -1200;
  calibration.joints[0].max_tick = 300;
  calibration.joints[0].home_tick = -400;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  head_state_init(&runtime);
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = true;

  /* A homing reference from earlier in this boot is dropped: q then maps to
   * the profile's raw range, and the range generation changes. */
  runtime.homing_reference_valid[0] = true;
  runtime.homing_zero_tick[0] = 5000;
  assert(head_control_position_range(&runtime, &calibration, 0u, &minimum, &maximum));
  assert(minimum == 4200 && maximum == 5700);
  runtime.servos[0].present_tick = -1200;
  const uint8_t generation = runtime.position_range_generation;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == 0);
  assert(!runtime.homing_reference_valid[0]);
  assert(runtime.position_range_generation != generation);
  assert(head_control_position_range(&runtime, &calibration, 0u, &minimum, &maximum));
  assert(minimum == -1200 && maximum == 300);
  assert(runtime.state == HEAD_HOMING_REQUIRED);  /* preparation changes state */
  runtime.servos[0].present_tick = 300;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == 0);

  /* Outside the range is refused, never clamped. */
  runtime.servos[0].present_tick = -1201;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == -ERANGE);
  runtime.servos[0].present_tick = 301;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == -ERANGE);
  runtime.servos[0].present_tick = 0;

  /* Only from torque-off HOMING_REQUIRED with motion preparation allowed. */
  runtime.state = HEAD_READY;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == -EPERM);
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == -EPERM);
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = false;
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == -EPERM);
  runtime.discovery_verified = true;
  head_state_fault(&runtime, HEAD_FAULT_BUS);
  assert(head_state_request_enable_at_present_pose(&runtime, &calibration) == -EPERM);

  /* Reinitialising control (profile applied) also changes the generation. */
  const uint8_t before_init = runtime.position_range_generation;
  head_control_init(&runtime, &calibration);
  assert(runtime.position_range_generation != before_init);
}

/* The lag limit follows the typed current limit, so the servo can be loaded
 * up to that limit; a low limit keeps the zero-home window as its floor. */
static void test_tuning_lag_limit_follows_current_limit(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime = {0};
  make_valid_calibration(&calibration);
  calibration.joints[0].operating_current_ma = 600;
  head_config_finalize(&calibration);
  assert(head_config_validate(&calibration));
  head_state_init(&runtime);
  runtime.state = HEAD_HOMING_REQUIRED;
  runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  runtime.discovery_verified = true;
  assert(head_state_request_tuning(&runtime, &calibration, 399u));
  runtime.state = HEAD_TUNING;
  runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
  runtime.tuning_neutral_tick[0] = 1000;
  runtime.servos[0].last_current_feedback_ms = 500u;

  /* 399 / 3 = 133 ticks. At 132 ticks of lag (about 400 mA measured, but
   * under the limit here) a step away is still accepted... */
  runtime.servos[0].goal_tick = 1300;
  runtime.servos[0].present_tick = 1300 - 132;
  runtime.servos[0].present_current_ma = 398;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == 0);
  /* ...at 133 it is refused, and stepping back still works. */
  runtime.servos[0].goal_tick = 1300;
  runtime.servos[0].present_tick = 1300 - 133;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EBUSY);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, -1, 510u) == 0);
  /* The old fixed 20-tick window no longer stops a loaded servo. */
  runtime.servos[0].goal_tick = 1300;
  runtime.servos[0].present_tick = 1300 - 70;
  runtime.servos[0].present_current_ma = 245;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 50, 510u) == 0);
  /* Reaching the typed limit still refuses, whatever the lag. */
  runtime.servos[0].present_tick = runtime.servos[0].goal_tick;
  runtime.servos[0].present_current_ma = 399;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EBUSY);

  /* A low limit keeps the floor: 30 / 3 = 10 is raised to 20 ticks. */
  runtime.tuning_current_limit_ma = 30u;
  runtime.servos[0].present_current_ma = 0;
  runtime.servos[0].goal_tick = 1300;
  runtime.servos[0].present_tick = 1300 - (HEAD_TUNING_MIN_LAG_TICKS - 1);
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == 0);
  runtime.servos[0].goal_tick = 1300;
  runtime.servos[0].present_tick = 1300 - HEAD_TUNING_MIN_LAG_TICKS;
  assert(head_state_tuning_step(&runtime, &calibration, 0u, 1, 510u) == -EBUSY);
}

int main(void)
{
  test_tuning_lag_limit_follows_current_limit();
  test_enable_seeds_requested_positions_exactly();
  test_enable_at_present_pose_request();
  test_non_finite_calibration_is_rejected();
  test_branch_current_budget();
  test_profile_declares_branch();
  test_tuning_request_and_steps();
  test_tuning_per_servo_limit();
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
  test_zero_homing_reaches_encoder_zero();
  test_fault_latch_and_disable_recovery_gate();
  test_fault_shutdown_is_idempotent();
  test_torque_never_commanded_on_gate();
  test_storage_excludes_motion_preparation();
  test_shifted_homing_reference_position_boundaries();
  return 0;
}
