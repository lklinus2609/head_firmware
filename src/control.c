#include <string.h>

#include "config.h"
#include "control.h"
#include "state_machine.h"

static bool is_active(const struct head_calibration *calibration, uint8_t index)
{
  return (calibration->active_servo_mask & (1u << index)) != 0u;
}

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
  if (value < low) return low;
  return value > high ? high : value;
}

static uint64_t stopping_distance(uint32_t speed, uint32_t acceleration)
{
  const uint64_t quotient = speed / acceleration;
  const uint64_t remainder = speed % acceleration;
  return acceleration * quotient * (quotient + 1u) / 2u +
         remainder * (quotient + 1u);
}

static int32_t stopping_speed_for_distance(uint32_t distance, int32_t max_step_ticks_per_control_cycle,
                                           int32_t max_acceleration_ticks_per_control_cycle_squared)
{
  uint32_t low = 0u;
  uint32_t high = (uint32_t)max_step_ticks_per_control_cycle;
  const uint32_t acceleration = (uint32_t)max_acceleration_ticks_per_control_cycle_squared;

  while (low < high) {
    const uint32_t middle = low + (high - low + 1u) / 2u;
    if (stopping_distance(middle, acceleration) <= distance) {
      low = middle;
    } else {
      high = middle - 1u;
    }
  }
  return (int32_t)low;
}

int32_t head_control_trajectory_step(int32_t position, int32_t previous_step,
                                     int32_t target, int32_t max_step_ticks_per_control_cycle,
                                     int32_t max_acceleration_ticks_per_control_cycle_squared)
{
  if (max_step_ticks_per_control_cycle <= 0 || max_acceleration_ticks_per_control_cycle_squared <= 0) return 0;

  const int64_t error = (int64_t)target - position;
  const uint32_t distance = (uint32_t)(error < 0 ? -error : error);
  const int32_t stopping_speed = stopping_speed_for_distance(distance, max_step_ticks_per_control_cycle,
                                                              max_acceleration_ticks_per_control_cycle_squared);
  const int32_t desired_step = error > 0 ? stopping_speed :
                               error < 0 ? -stopping_speed : 0;
  const int64_t delta = (int64_t)desired_step - previous_step;
  const int32_t bounded_delta = delta < -max_acceleration_ticks_per_control_cycle_squared ? -max_acceleration_ticks_per_control_cycle_squared :
                                delta > max_acceleration_ticks_per_control_cycle_squared ? max_acceleration_ticks_per_control_cycle_squared : (int32_t)delta;
  const int64_t next = (int64_t)previous_step + bounded_delta;
  if (next < -max_step_ticks_per_control_cycle) return -max_step_ticks_per_control_cycle;
  if (next > max_step_ticks_per_control_cycle) return max_step_ticks_per_control_cycle;
  return (int32_t)next;
}

static float clamp_unit(float value)
{
  /* Keep this freestanding: the Zephyr/minimal-libc toolchain need not link
   * libm for the C99 isfinite macro.  NaN is the only float not equal to
   * itself; +/- infinity are safely handled by the bounds below. */
  if (value != value) {
    return 0.0f;
  }
  if (value < 0.0f) {
    return 0.0f;
  }
  return value > 1.0f ? 1.0f : value;
}

static bool finite_float(float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7F800000u) != 0x7F800000u;
}

void head_control_init(struct head_runtime *runtime,
                       const struct head_calibration *calibration)
{
  memset(runtime->servos, 0, sizeof(runtime->servos));
  memset(runtime->branches, 0, sizeof(runtime->branches));
  memset(runtime->homing_zero_tick, 0, sizeof(runtime->homing_zero_tick));
  memset(runtime->homing_origin_tick, 0, sizeof(runtime->homing_origin_tick));
  memset(runtime->homing_motion_remainder_ticks_per_second, 0,
         sizeof(runtime->homing_motion_remainder_ticks_per_second));
  memset(runtime->homing_reference_valid, 0,
         sizeof(runtime->homing_reference_valid));
  head_control_note_position_range_changed(runtime);
  runtime->last_command_ms = 0u;
  runtime->watchdog_hold_started_ms = 0u;
  runtime->last_accepted_sequence = 0u;
  runtime->last_transmitted_sequence = 0u;
  runtime->accepted_sequence_valid = false;
  runtime->homing_torque_index = UINT8_MAX;
  runtime->homing_backoff_active = false;
  runtime->pending_transmit_valid = false;
  runtime->pending_transmit_branch_mask = 0u;
  runtime->pending_transmit_sequence = 0u;
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    runtime->servos[servo_index].goal_tick = calibration->joints[servo_index].home_tick;
    runtime->requested_position_normalized[servo_index] = 0.5f;
  }
}

bool head_control_accept_command(struct head_runtime *runtime,
                                 const struct head_command *command,
                                 const struct head_calibration *calibration,
                                 uint32_t now_ms)
{
  if (runtime->state != HEAD_ENABLED || command->lease_token == 0u ||
      command->lease_token != runtime->active_lease_token ||
      (int32_t)(now_ms - runtime->lease_expires_ms) >= 0 ||
      command->active_servo_mask != calibration->active_servo_mask ||
      command->mode != HEAD_COMMAND_POSITION ||
      (runtime->accepted_sequence_valid &&
       (int32_t)(command->sequence - runtime->last_accepted_sequence) <= 0)) {
    return false;
  }
  for (uint8_t servo_index = 0u; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if (!is_active(calibration, servo_index)) continue;
    if (!finite_float(command->position_normalized[servo_index]) ||
        command->position_normalized[servo_index] < 0.0f ||
        command->position_normalized[servo_index] > 1.0f ||
        !finite_float(command->velocity_normalized[servo_index])) {
      return false;
    }
  }
  runtime->last_command_ms = now_ms;
  runtime->last_accepted_sequence = command->sequence;
  runtime->accepted_sequence_valid = true;
  runtime->watchdog_hold_started_ms = 0u;
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if (is_active(calibration, servo_index)) {
      runtime->requested_position_normalized[servo_index] = command->position_normalized[servo_index];
    }
  }
  return true;
}

bool head_control_position_range(const struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t servo_index, int32_t *minimum_tick,
                                 int32_t *maximum_tick)
{
  if (servo_index >= HEAD_SERVO_COUNT) return false;
  const struct head_joint_config *joint = &calibration->joints[servo_index];
  const int64_t reference = runtime->homing_reference_valid[servo_index] ?
      runtime->homing_zero_tick[servo_index] : joint->home_tick;
  const int64_t minimum = reference + (int64_t)joint->min_tick - joint->home_tick;
  const int64_t maximum = reference + (int64_t)joint->max_tick - joint->home_tick;
  if (minimum < HEAD_DXL_POSITION_MIN_TICK || maximum > HEAD_DXL_POSITION_MAX_TICK ||
      minimum >= maximum) {
    return false;
  }
  if (minimum_tick != NULL) *minimum_tick = (int32_t)minimum;
  if (maximum_tick != NULL) *maximum_tick = (int32_t)maximum;
  return true;
}

bool head_control_reference_valid(const struct head_runtime *runtime,
                                  const struct head_calibration *calibration,
                                  uint8_t servo_index)
{
  return head_control_position_range(runtime, calibration, servo_index, NULL, NULL);
}

void head_control_note_position_range_changed(struct head_runtime *runtime)
{
  ++runtime->position_range_generation;
}

int32_t head_control_target_tick(const struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t servo_index)
{
  int32_t minimum;
  int32_t maximum;
  if (!head_control_position_range(runtime, calibration, servo_index, &minimum, &maximum)) {
    return 0;
  }
  const int32_t target = minimum + (int32_t)((float)(maximum - minimum) *
                         clamp_unit(runtime->requested_position_normalized[servo_index]));
  return clamp_i32(target, minimum, maximum);
}

/* On entry to ENABLED the requested q must be the pose the servos are in.
 * Otherwise head_control_tick() drives toward whatever q was left over (0.5
 * from boot, or the previous session's last target) before the host has sent
 * anything. Half a tick is added before dividing so that the float product in
 * head_control_target_tick() truncates back to the same tick instead of one
 * below it; the result is checked rather than trusted. */
bool head_control_seed_requested_positions(struct head_runtime *runtime,
                                           const struct head_calibration *calibration)
{
  for (uint8_t servo_index = 0u; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if (!is_active(calibration, servo_index)) continue;
    int32_t minimum;
    int32_t maximum;
    const int32_t goal = runtime->servos[servo_index].goal_tick;
    if (!head_control_position_range(runtime, calibration, servo_index, &minimum, &maximum) ||
        goal < minimum || goal > maximum) {
      return false;
    }
    runtime->requested_position_normalized[servo_index] = clamp_unit(
        ((float)(goal - minimum) + 0.5f) / (float)(maximum - minimum));
    if (head_control_target_tick(runtime, calibration, servo_index) != goal) return false;
  }
  return true;
}

uint32_t head_control_present_current_branch_ma(
    const struct head_runtime *runtime,
    const struct head_calibration *calibration,
    uint8_t branch_index)
{
  uint32_t total_current_ma = 0u;
  uint8_t servo_indexes[HEAD_SERVOS_PER_BRANCH];
  if (runtime == NULL || calibration == NULL || branch_index >= HEAD_BRANCH_COUNT) {
    return UINT32_MAX;
  }
  const uint8_t servo_count =
      head_config_branch_servo_indexes(calibration, branch_index, servo_indexes);
  for (uint8_t member = 0u; member < servo_count; ++member) {
    const int32_t current_ma = runtime->servos[servo_indexes[member]].present_current_ma;
    total_current_ma += (uint32_t)(current_ma < 0 ? -current_ma : current_ma);
  }
  return total_current_ma;
}

void head_control_tick(struct head_runtime *runtime,
                       const struct head_calibration *calibration,
                       uint32_t now_ms)
{
  if (runtime->state != HEAD_ENABLED && runtime->state != HEAD_ROUTING) {
    return;
  }
  if (runtime->state == HEAD_ENABLED &&
      (now_ms - runtime->last_command_ms) > HEAD_COMMAND_TIMEOUT_MS) {
    if (runtime->watchdog_hold_started_ms == 0u) {
      runtime->watchdog_hold_started_ms = now_ms;
      return;
    }
    if ((now_ms - runtime->watchdog_hold_started_ms) >= HEAD_TIMEOUT_HOLD_MS) {
      /* A lost command stream is a latched safety event.  Do not leave the
       * controller in READY with an uncleared fault bit. */
      head_state_fault(runtime, HEAD_FAULT_WATCHDOG);
    }
    return;
  }

  /* Command values are staged by the protocol dispatcher in main.c. */
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if (!is_active(calibration, servo_index)) continue;
    const struct head_joint_config *joint = &calibration->joints[servo_index];
    if (!head_control_reference_valid(runtime, calibration, servo_index)) {
      head_state_fault(runtime, HEAD_FAULT_CONFIGURATION);
      return;
    }
    const int32_t desired = runtime->state == HEAD_ROUTING ?
                            HEAD_ROUTING_BASE_TICK :
                            head_control_target_tick(runtime, calibration, servo_index);
    const int32_t range = joint->max_tick - joint->min_tick;
    const int32_t configured_max_step_ticks_per_control_cycle = (int32_t)(
        (float)range * joint->max_position_fraction_per_control_cycle);
    const int32_t configured_max_acceleration_ticks_per_control_cycle_squared = (int32_t)(
        (float)range * joint->max_acceleration_fraction_per_control_cycle_squared);
    const int32_t max_step_ticks_per_control_cycle = configured_max_step_ticks_per_control_cycle > 0 ? configured_max_step_ticks_per_control_cycle : 1;
    const int32_t max_acceleration_ticks_per_control_cycle_squared = configured_max_acceleration_ticks_per_control_cycle_squared > 0 ? configured_max_acceleration_ticks_per_control_cycle_squared : 1;
    const int32_t applied = head_control_trajectory_step(
        runtime->servos[servo_index].goal_tick, runtime->servos[servo_index].goal_step_ticks_per_control_cycle,
        desired, max_step_ticks_per_control_cycle, max_acceleration_ticks_per_control_cycle_squared);
    runtime->servos[servo_index].goal_step_ticks_per_control_cycle = applied;
    runtime->servos[servo_index].goal_tick += applied;
  }
}
