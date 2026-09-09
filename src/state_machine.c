#include <stdlib.h>

#include "config.h"
#include "control.h"
#include "dxl.h"
#include "state_machine.h"

void head_state_init(struct head_runtime *runtime)
{
  runtime->state = HEAD_BOOT;
  runtime->fault = HEAD_FAULT_NONE;
  runtime->discovery_verified = false;
  runtime->discovery_active = false;
  runtime->preparation_active = false;
  runtime->diagnostic_active = false;
  runtime->diagnostic_cancel_requested = false;
  runtime->torque_state = HEAD_TORQUE_UNKNOWN;
  runtime->shutdown_requested = false;
  runtime->active_lease_token = 0u;
  runtime->lease_expires_ms = 0u;
  runtime->homing_index = 0u;
  runtime->proprioception_started_ms = 0u;
  runtime->homing_torque_index = UINT8_MAX;
  runtime->homing_backoff_active = false;
  runtime->maintenance_calibration = false;
  runtime->maintenance_waiting_confirm = false;
}

bool head_state_request_home(struct head_runtime *runtime,
                             const struct head_calibration *calibration)
{
  if (runtime->state != HEAD_HOMING_REQUIRED ||
      !head_state_can_prepare_motion(runtime) ||
      !head_config_validate(calibration)) {
    return false;
  }
  runtime->state = HEAD_HOMING;
  runtime->homing_index = 0u;
  runtime->homing_started_ms = 0u;
  runtime->homing_qualified_since_ms = 0u;
  runtime->homing_backoff_active = false;
  runtime->maintenance_calibration = false;
  runtime->maintenance_waiting_confirm = false;
  return true;
}

bool head_state_request_maintenance_calibration(struct head_runtime *runtime,
                                                const struct head_calibration *calibration)
{
  if (runtime->state != HEAD_HOMING_REQUIRED ||
      !head_state_can_prepare_motion(runtime) || !head_config_validate(calibration)) {
    return false;
  }
  runtime->state = HEAD_MAINTENANCE_CALIBRATION;
  runtime->maintenance_calibration = true;
  runtime->homing_index = 0u;
  runtime->homing_started_ms = 0u;
  runtime->homing_qualified_since_ms = 0u;
  runtime->homing_backoff_active = false;
  runtime->maintenance_waiting_confirm = false;
  return true;
}

bool head_state_confirm_maintenance_servo(struct head_runtime *runtime,
                                          struct head_calibration *calibration,
                                          uint8_t servo_index)
{
  if (runtime->state != HEAD_MAINTENANCE_CALIBRATION ||
      !runtime->maintenance_waiting_confirm || servo_index != runtime->homing_index ||
      servo_index >= HEAD_SERVO_COUNT) return false;
  calibration->joints[servo_index].home_tick = runtime->homing_zero_tick[servo_index];
  head_config_finalize(calibration);
  ++runtime->homing_index;
  runtime->homing_started_ms = 0u;
  runtime->homing_qualified_since_ms = 0u;
  runtime->maintenance_waiting_confirm = false;
  runtime->homing_backoff_active = false;
  return true;
}

bool head_state_request_enable(struct head_runtime *runtime)
{
  if (runtime->fault != HEAD_FAULT_NONE || runtime->state != HEAD_READY || !head_state_torque_verified_on(runtime)) return false;
  runtime->state = HEAD_ENABLED;
  runtime->last_command_ms = 0u;
  runtime->watchdog_hold_started_ms = 0u;
  return true;
}

bool head_state_request_proprioception(struct head_runtime *runtime,
                                       uint32_t now_ms)
{
  if (runtime->fault != HEAD_FAULT_NONE || runtime->state != HEAD_READY || !head_state_torque_verified_on(runtime)) {
    return false;
  }
  runtime->state = HEAD_PROPRIOCEPTION_SETTLING;
  runtime->proprioception_started_ms = now_ms;
  runtime->last_command_ms = 0u;
  runtime->watchdog_hold_started_ms = 0u;
  runtime->accepted_sequence_valid = false;
  runtime->pending_transmit_valid = false;
  return true;
}

bool head_state_request_routing(struct head_runtime *runtime,
                                const struct head_calibration *calibration)
{
  if ((runtime->state != HEAD_HOMING_REQUIRED &&
       runtime->state != HEAD_READY) ||
      !head_state_torque_verified_on(runtime) ||
      !head_config_validate(calibration)) {
    return false;
  }
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((calibration->active_servo_mask & (1u << index)) == 0u) continue;
    const struct head_joint_config *joint = &calibration->joints[index];
    if (HEAD_ROUTING_BASE_TICK < joint->min_tick ||
        HEAD_ROUTING_BASE_TICK > joint->max_tick) {
      return false;
    }
  }
  runtime->state = HEAD_ROUTING;
  runtime->last_command_ms = 0u;
  runtime->watchdog_hold_started_ms = 0u;
  runtime->accepted_sequence_valid = false;
  runtime->pending_transmit_valid = false;
  return true;
}

bool head_state_torque_may_be_on(const struct head_runtime *runtime)
{
  return runtime->torque_state == HEAD_TORQUE_ON_VERIFIED ||
         runtime->torque_state == HEAD_TORQUE_SHUTDOWN_PENDING ||
         runtime->torque_state == HEAD_TORQUE_SHUTDOWN_FAILED;
}

bool head_state_torque_verified_on(const struct head_runtime *runtime)
{
  return runtime->torque_state == HEAD_TORQUE_ON_VERIFIED;
}

bool head_state_can_prepare_motion(const struct head_runtime *runtime)
{
  return runtime->fault == HEAD_FAULT_NONE && runtime->discovery_verified && !runtime->discovery_active &&
         runtime->torque_state == HEAD_TORQUE_OFF_VERIFIED &&
         !runtime->shutdown_requested && !runtime->preparation_active &&
         !runtime->diagnostic_active &&
         runtime->storage_state != HEAD_STORAGE_PENDING &&
         runtime->storage_state != HEAD_STORAGE_WRITING;
}

static void request_shutdown(struct head_runtime *runtime)
{
  if (!runtime->shutdown_requested) {
    runtime->shutdown_confirmations = 0u;
    runtime->shutdown_next_attempt_ms = 0u;
    runtime->torque_state = HEAD_TORQUE_SHUTDOWN_PENDING;
  }
  runtime->shutdown_requested = true;
  runtime->diagnostic_cancel_requested = runtime->diagnostic_active;
  runtime->preparation_active = false;
  runtime->active_lease_token = 0u;
  runtime->accepted_sequence_valid = false;
  runtime->pending_transmit_valid = false;
}

void head_state_disable(struct head_runtime *runtime)
{
  if (runtime->fault == HEAD_FAULT_NONE && runtime->state != HEAD_FAULT) {
    runtime->state = HEAD_HOMING_REQUIRED;
  }
  request_shutdown(runtime);
}

void head_state_fault(struct head_runtime *runtime, enum head_fault fault)
{
  if (runtime->fault == HEAD_FAULT_NONE) runtime->fault = fault;
  runtime->state = HEAD_FAULT;
  request_shutdown(runtime);
}

static void finish_homing(struct head_runtime *runtime)
{
  /* Successful homing retains its live owner for the following enable/hold
   * request. Disable and faults still revoke ownership immediately. */
  const uint32_t lease_token = runtime->active_lease_token;
  runtime->state = HEAD_READY;
  runtime->maintenance_calibration = false;
  request_shutdown(runtime);
  runtime->active_lease_token = lease_token;
}

static void set_homing_goal(struct head_runtime *runtime,
                            const struct head_joint_config *joint,
                            uint8_t servo_index, int64_t goal_tick)
{
  if (goal_tick < HEAD_DXL_POSITION_MIN_TICK || goal_tick > HEAD_DXL_POSITION_MAX_TICK ||
      llabs(goal_tick - runtime->homing_origin_tick[servo_index]) > joint->homing_max_travel_ticks) {
    head_state_fault(runtime, HEAD_FAULT_HOMING);
    return;
  }
  runtime->servos[servo_index].goal_tick = (int32_t)goal_tick;
}

void head_state_homing_tick(struct head_runtime *runtime,
                            const struct head_calibration *calibration,
                            uint32_t now_ms)
{
  uint8_t index = runtime->homing_index;
  const struct head_joint_config *joint;
  struct head_servo_state *servo;
  int64_t error;

  if (runtime->state != HEAD_HOMING &&
      runtime->state != HEAD_MAINTENANCE_CALIBRATION) return;
  while (index < HEAD_SERVO_COUNT &&
         (calibration->active_servo_mask & (1u << index)) == 0u) {
    ++index;
  }
  runtime->homing_index = index;
  const uint8_t active_index = runtime->homing_index;
  if (active_index >= HEAD_SERVO_COUNT) {
    finish_homing(runtime);
    return;
  }
  joint = &calibration->joints[active_index];
  servo = &runtime->servos[active_index];
  if (servo->present_tick < HEAD_DXL_POSITION_MIN_TICK ||
      servo->present_tick > HEAD_DXL_POSITION_MAX_TICK) {
    head_state_fault(runtime, HEAD_FAULT_HOMING);
    return;
  }
  if (runtime->homing_started_ms == 0u) {
    /* Start from fresh feedback so torque-on never causes a jump to a stored
     * homing position. */
    servo->goal_tick = servo->present_tick;
    servo->goal_step_ticks_per_control_cycle = 0;
    runtime->homing_origin_tick[active_index] = servo->present_tick;
    runtime->homing_started_ms = now_ms;
    runtime->homing_qualified_since_ms = 0u;
    runtime->homing_motion_remainder_ticks_per_second[active_index] = 0u;
    runtime->homing_backoff_active = false;
    return;
  }
  if (runtime->maintenance_waiting_confirm) return;
  if (!servo->online || now_ms - servo->last_feedback_ms > 20u ||
      (now_ms - runtime->homing_started_ms) > joint->homing_timeout_ms ||
      llabs((int64_t)servo->goal_tick - runtime->homing_origin_tick[active_index]) >
          joint->homing_max_travel_ticks) {
    head_state_fault(runtime, HEAD_FAULT_HOMING);
    return;
  }
  error = llabs((int64_t)servo->goal_tick - servo->present_tick);
  runtime->homing_motion_remainder_ticks_per_second[active_index] +=
      joint->homing_speed_ticks_per_second;
  int32_t motion_step = (int32_t)(runtime->homing_motion_remainder_ticks_per_second[active_index] /
                                  HEAD_CONTROL_HZ);
  runtime->homing_motion_remainder_ticks_per_second[active_index] %= HEAD_CONTROL_HZ;
  if (runtime->homing_backoff_active) {
    const int64_t backed_off = llabs((int64_t)servo->goal_tick -
                                   runtime->homing_zero_tick[active_index]);
    if (backed_off < joint->homing_backoff_ticks) {
      const int32_t remaining_backoff = (int32_t)(joint->homing_backoff_ticks - backed_off);
      if (motion_step > remaining_backoff) motion_step = remaining_backoff;
      set_homing_goal(runtime, joint, active_index, (int64_t)servo->goal_tick +
          (joint->homing_direction > 0.0f ? -motion_step : motion_step));
      return;
    }
    const int32_t settled_limit = joint->homing_following_error_ticks > 2 ?
                                  joint->homing_following_error_ticks : 2;
    if (error > settled_limit) return;
    if (runtime->maintenance_calibration) {
      runtime->maintenance_waiting_confirm = true;
      return;
    }
    ++runtime->homing_index;
    runtime->homing_started_ms = 0u;
    runtime->homing_qualified_since_ms = 0u;
    runtime->homing_backoff_active = false;
    /* Complete/skip inactive slots now, before the control owner performs a handoff. */
    while (runtime->homing_index < HEAD_SERVO_COUNT &&
           (calibration->active_servo_mask & (1u << runtime->homing_index)) == 0u) {
      ++runtime->homing_index;
    }
    if (runtime->homing_index >= HEAD_SERVO_COUNT) {
      finish_homing(runtime);
    }
    return;
  }
  if (abs(servo->present_current_ma) >= joint->homing_current_ma &&
      (joint->homing_following_error_ticks == 0 ||
       error >= joint->homing_following_error_ticks)) {
    if (runtime->homing_qualified_since_ms == 0u) {
      runtime->homing_qualified_since_ms = now_ms;
    } else if ((now_ms - runtime->homing_qualified_since_ms) >= joint->homing_persistence_ms) {
      runtime->homing_zero_tick[active_index] = servo->present_tick;
      runtime->homing_reference_valid[active_index] = true;
      if (!head_control_reference_valid(runtime, calibration, active_index)) {
        head_state_fault(runtime, HEAD_FAULT_HOMING);
        return;
      }
      servo->goal_tick = servo->present_tick;
      runtime->homing_qualified_since_ms = 0u;
      runtime->homing_motion_remainder_ticks_per_second[active_index] = 0u;
      runtime->homing_backoff_active = true;
      return;
    }
  } else {
    runtime->homing_qualified_since_ms = 0u;
  }
  set_homing_goal(runtime, joint, active_index, (int64_t)servo->goal_tick +
      (joint->homing_direction > 0.0f ? motion_step : -motion_step));
}

void head_state_proprioception_tick(struct head_runtime *runtime,
                                    uint32_t now_ms)
{
  if (runtime->state != HEAD_PROPRIOCEPTION_SETTLING) return;
  /* This is a minimum quieting interval, not a claim that silicone creep has
   * ended. The data-collection host may discard a longer warm-up interval. */
  if ((now_ms - runtime->proprioception_started_ms) >=
      HEAD_PROPRIOCEPTION_SETTLE_MS) {
    runtime->state = HEAD_PROPRIOCEPTION_HOLD;
  }
}
