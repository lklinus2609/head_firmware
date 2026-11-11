#include <errno.h>
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
  runtime->zero_homing = false;
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
  runtime->zero_homing = false;
  return true;
}

/* A joint with no mechanical stop cannot be homed by searching for one: there
 * is nothing to push against, so stop detection either never fires or fires on
 * a friction transient and records an arbitrary zero. The absolute encoder's
 * own zero is the alternative datum -- a fixed physical angle of the output
 * shaft, repeatable within a revolution. */
bool head_state_request_zero_home(struct head_runtime *runtime,
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
  runtime->zero_homing = true;
  return true;
}

/* The datum: mid-revolution, tick 2048.
 *
 * Any fixed point in the encoder's single turn is equally repeatable, because
 * the encoder is absolute within a revolution -- so the datum should be chosen
 * for where the servo holds well, not for being numerically zero. It holds
 * badly on its Position Limit boundaries, which are 0 and 4095 from the
 * factory and which nothing here changes: a goal of 4096 sits one tick above
 * the maximum and never settles, and a goal of 0 sits exactly on the minimum,
 * so every overshoot lands outside the permitted range and is fought back.
 * Measured on the bench, both oscillate about 150 ticks either side drawing
 * several hundred milliamps. Mid-revolution has 2048 ticks of margin on both
 * sides, so an overshoot stays inside the range and is simply corrected. */
static int32_t encoder_datum_tick(void)
{
  /* The encoder's own zero. Tick 4096 is the same shaft angle but sits one
   * tick above Max Position Limit, which the servo will not hold, so 0 is the
   * expressible one. It is exactly on Min Position Limit, so an overshoot
   * lands outside the permitted range -- tolerable now that Position D damps
   * the approach, but the reason a mid-revolution datum was tried first. */
  return 0;
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
  head_control_note_position_range_changed(runtime);
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

/* ENABLED without homing, for a head whose ranges were found by tuning and
 * written to the profile. Homing moves servos, which a tensioned head must not
 * do, so q is mapped to the profile's raw min_tick..max_tick instead of to a
 * homing reference. That is only meaningful when every active servo is already
 * inside its range, so anything else is refused with -ERANGE rather than
 * clamped: clamping would command a move to the range end at Enable. Any
 * homing reference left from earlier in this boot is dropped. The caller then
 * prepares torque with HEAD_ENABLED as the target, and preparation completion
 * seeds q from the freshly read positions and checks the range again. */
int head_state_request_enable_at_present_pose(struct head_runtime *runtime,
                                              const struct head_calibration *calibration)
{
  if (runtime->state != HEAD_HOMING_REQUIRED ||
      !head_state_can_prepare_motion(runtime) ||
      !head_config_validate(calibration)) {
    return -EPERM;
  }
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((calibration->active_servo_mask & (1u << index)) == 0u) continue;
    const struct head_joint_config *joint = &calibration->joints[index];
    const int32_t present_tick = runtime->servos[index].present_tick;
    if (joint->min_tick >= joint->max_tick ||
        present_tick < joint->min_tick || present_tick > joint->max_tick) {
      return -ERANGE;
    }
  }
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    runtime->homing_reference_valid[index] = false;
  }
  head_control_note_position_range_changed(runtime);
  return 0;
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

/* Tuning starts from HOMING_REQUIRED without homing: limits and tensions are
 * not known yet, which is what tuning is for. The caller prepares torque with
 * HEAD_TUNING as the target, and preparation seeds every goal from the present
 * position, so entering tuning commands no motion. Each active servo's Goal
 * Current is the requested limit or its own commissioned operating_current_ma,
 * whichever is lower (head_state_tuning_limit_ma), so one joint can be allowed
 * more than the others: on 2026-10-20 the jaw (7) got 650 mA peaks while the
 * modiolus lines stayed at 300 mA. The request must be within the highest
 * operating_current_ma; a request above every servo's would cap nothing. */
bool head_state_request_tuning(struct head_runtime *runtime,
                               const struct head_calibration *calibration,
                               uint16_t current_limit_ma)
{
  if (runtime->state != HEAD_HOMING_REQUIRED ||
      !head_state_can_prepare_motion(runtime) ||
      !head_config_validate(calibration) || current_limit_ma == 0u) {
    return false;
  }
  uint16_t highest_ma = 0u;
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((calibration->active_servo_mask & (1u << index)) == 0u) continue;
    if (calibration->joints[index].operating_current_ma <= 0) return false;
    if ((uint16_t)calibration->joints[index].operating_current_ma > highest_ma) {
      highest_ma = (uint16_t)calibration->joints[index].operating_current_ma;
    }
  }
  if (current_limit_ma > highest_ma) return false;
  runtime->tuning_current_limit_ma = current_limit_ma;
  return true;
}

uint16_t head_state_tuning_limit_ma(const struct head_runtime *runtime,
                                    const struct head_calibration *calibration,
                                    uint8_t servo_index)
{
  const int16_t operating_ma = calibration->joints[servo_index].operating_current_ma;
  if (operating_ma <= 0) return 0u;
  return runtime->tuning_current_limit_ma < (uint16_t)operating_ma ?
      runtime->tuning_current_limit_ma : (uint16_t)operating_ma;
}

static int64_t tuning_max_lag_ticks(uint16_t current_limit_ma)
{
  const int64_t lag_ticks = current_limit_ma / HEAD_TUNING_LAG_CURRENT_MA_PER_TICK;
  return lag_ticks > HEAD_TUNING_MIN_LAG_TICKS ? lag_ticks : HEAD_TUNING_MIN_LAG_TICKS;
}

/* One operator click: move one servo by step_ticks (1 to
 * HEAD_TUNING_MAX_STEP_TICKS either way), toward or away from its neutral. A step toward neutral is always allowed, so a servo
 * that has reached its limit can always be backed off. A step away from
 * neutral is refused at one turn of travel, or while the servo is saturated
 * at the current limit or trailing its goal -- the mechanism is resisting. */
int head_state_tuning_step(struct head_runtime *runtime,
                           const struct head_calibration *calibration,
                           uint8_t servo_index, int8_t step_ticks, uint32_t now_ms)
{
  if (runtime->state != HEAD_TUNING || runtime->fault != HEAD_FAULT_NONE ||
      !head_state_torque_verified_on(runtime) || runtime->shutdown_requested) {
    return -EPERM;
  }
  if (servo_index >= HEAD_SERVO_COUNT ||
      (calibration->active_servo_mask & (1u << servo_index)) == 0u ||
      step_ticks == 0 || step_ticks > HEAD_TUNING_MAX_STEP_TICKS ||
      step_ticks < -HEAD_TUNING_MAX_STEP_TICKS) {
    return -EINVAL;
  }
  struct head_servo_state *servo = &runtime->servos[servo_index];
  if (servo->last_current_feedback_ms == 0u ||
      now_ms - servo->last_current_feedback_ms > HEAD_COMMAND_FEEDBACK_FRESH_MS) {
    return -EAGAIN;
  }
  const int64_t neutral = runtime->tuning_neutral_tick[servo_index];
  const int64_t goal = (int64_t)servo->goal_tick + step_ticks;
  const int64_t travel = llabs(goal - neutral);
  if (goal < HEAD_DXL_POSITION_MIN_TICK || goal > HEAD_DXL_POSITION_MAX_TICK ||
      travel > HEAD_TUNING_MAX_TRAVEL_TICKS) {
    return -ERANGE;
  }
  const uint16_t limit_ma = head_state_tuning_limit_ma(runtime, calibration, servo_index);
  if (travel > llabs((int64_t)servo->goal_tick - neutral) &&
      (abs(servo->present_current_ma) >= limit_ma ||
       llabs((int64_t)servo->goal_tick - servo->present_tick) >=
           tuning_max_lag_ticks(limit_ma))) {
    return -EBUSY;
  }
  servo->goal_tick = (int32_t)goal;
  servo->goal_step_ticks_per_control_cycle = 0;
  return 0;
}

bool head_state_torque_may_be_on(const struct head_runtime *runtime)
{
  return runtime->torque_state == HEAD_TORQUE_ON_VERIFIED ||
         runtime->torque_state == HEAD_TORQUE_SHUTDOWN_PENDING ||
         runtime->torque_state == HEAD_TORQUE_SHUTDOWN_FAILED;
}

/* True when this session has never commanded torque on, so nothing on the bus
 * can be energized by this firmware. Every torque-on route runs through
 * preparation, which requires head_state_can_prepare_motion() and therefore
 * discovery_verified; a failed discovery leaves that false. The shutdown sweep
 * cannot confirm itself against servos that never answer, so it parks
 * torque_state at SHUTDOWN_PENDING/FAILED permanently, which would otherwise
 * lock out both the read-only diagnostics and the discovery retry needed to
 * find out why the bus is silent. ON_VERIFIED is refused outright: once a
 * servo has been energized, only a verified sweep may call it safe. */
bool head_state_torque_never_commanded_on(const struct head_runtime *runtime)
{
  return runtime->state == HEAD_FAULT && !runtime->discovery_verified &&
         runtime->torque_state != HEAD_TORQUE_ON_VERIFIED;
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
  runtime->zero_homing = false;
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
  /* Feedback age alone decides this, not servo->online. Telemetry clears
   * online after a single timed-out Sync Read and sets it again on the next
   * good one, so online flaps for one cycle on any lost reply -- and the old
   * 20 ms companion window had no margin either, since a 10 ms poll with an
   * 8 ms response timeout is already ~18 ms behind after one miss. Homing a
   * joint with no mechanical stop sweeps for seconds rather than the 200 ms a
   * stop search took, so that near-certainly aborts a healthy run. Age is the
   * honest measure of whether the position being acted on is still true, and
   * the window below still catches feedback that has genuinely stopped. */
  if (now_ms - servo->last_feedback_ms > HEAD_HOMING_FEEDBACK_STALE_MS ||
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
  if (runtime->zero_homing) {
    /* Drive to the encoder zero nearest where this servo started, then take it
     * as the reference. set_homing_goal() still enforces the travel budget and
     * the position range, and the timeout and feedback-freshness checks above
     * apply unchanged, so this path is bounded exactly like a stop search. */
    /* The travel budget bounds the goal that is commanded, not where the servo
     * actually ends up -- a profile restarted on every rewrite once carried a
     * 977-tick command five thousand ticks past its target. Bound the real
     * deviation too, so a servo that stops obeying its goal is caught by the
     * position it reports rather than by the position it was asked for. */
    if (llabs((int64_t)servo->present_tick - (int64_t)servo->goal_tick) >
        joint->homing_max_travel_ticks) {
      head_state_fault(runtime, HEAD_FAULT_HOMING);
      return;
    }
    const int32_t target = encoder_datum_tick();
    if (servo->goal_tick != target) {
      /* Commanded once, not walked a tick at a time: preparation gave the servo
       * a Profile Acceleration and Velocity for this run, so its own generator
       * shapes the trapezoid. set_homing_goal() still enforces the travel
       * budget and the position range against this single destination. */
      set_homing_goal(runtime, joint, active_index, target);
      return;
    }
    /* Commanded position reached; wait for the servo to actually settle on it
     * before recording the datum, or the reference captures the lag. */
    /* Arrival tolerance for the datum. The stop-search fallback of 2 ticks is
     * 0.18 degrees, tighter than this servo holds with Position P 900 and no
     * I or D term, so waiting for it parks the servo on target hunting until
     * the homing timeout instead of declaring arrival. */
    const int32_t zero_settled_limit =
        joint->homing_following_error_ticks > HEAD_ZERO_HOME_SETTLE_TICKS ?
        joint->homing_following_error_ticks : HEAD_ZERO_HOME_SETTLE_TICKS;
    if (error > zero_settled_limit) return;
    runtime->homing_zero_tick[active_index] = target;
    runtime->homing_reference_valid[active_index] = true;
    head_control_note_position_range_changed(runtime);
    if (!head_control_reference_valid(runtime, calibration, active_index)) {
      head_state_fault(runtime, HEAD_FAULT_HOMING);
      return;
    }
    ++runtime->homing_index;
    runtime->homing_started_ms = 0u;
    runtime->homing_qualified_since_ms = 0u;
    runtime->homing_backoff_active = false;
    while (runtime->homing_index < HEAD_SERVO_COUNT &&
           (calibration->active_servo_mask & (1u << runtime->homing_index)) == 0u) {
      ++runtime->homing_index;
    }
    if (runtime->homing_index >= HEAD_SERVO_COUNT) {
      /* Arrive and hold, rather than finish_homing()'s torque release. A stop
       * search ends against a mechanical stop that holds the joint on its own;
       * an encoder-zero datum has nothing holding it, so releasing at the
       * moment of arrival lets inertia carry the joint straight back off the
       * position just established. HEAD_PROPRIOCEPTION_HOLD keeps torque on and
       * keeps the goals where they are without accepting motion commands, and
       * Disable remains the exit. READY cannot be used for this: it is a
       * torque-off state by contract, and every transition out of it requires
       * HEAD_TORQUE_OFF_VERIFIED. */
      runtime->state = HEAD_PROPRIOCEPTION_HOLD;
      runtime->last_command_ms = 0u;
      runtime->watchdog_hold_started_ms = 0u;
      runtime->accepted_sequence_valid = false;
      runtime->pending_transmit_valid = false;
    }
    return;
  }
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
      head_control_note_position_range_changed(runtime);
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
