#ifndef HEAD_CONTROL_H_
#define HEAD_CONTROL_H_

#include "head.h"

bool head_control_reference_valid(const struct head_runtime *runtime,
                                  const struct head_calibration *calibration,
                                  uint8_t servo_index);
/* The ticks that q = 0 and q = 1 map to: the homing reference when this servo
 * has one, otherwise the profile's raw min_tick..max_tick. False when that
 * range is empty or outside the Dynamixel position range. Either output
 * pointer may be NULL. */
bool head_control_position_range(const struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t servo_index, int32_t *minimum_tick,
                                 int32_t *maximum_tick);
/* Call whenever a position range may have changed. */
void head_control_note_position_range_changed(struct head_runtime *runtime);
/* Set every active servo's requested q to its present goal tick. False, with
 * nothing guaranteed about the requested values, when any active goal lies
 * outside its position range. */
bool head_control_seed_requested_positions(struct head_runtime *runtime,
                                           const struct head_calibration *calibration);
void head_control_init(struct head_runtime *runtime,
                       const struct head_calibration *calibration);
bool head_control_accept_command(struct head_runtime *runtime,
                                 const struct head_command *command,
                                 const struct head_calibration *calibration,
                                 uint32_t now_ms);
/* Return the next signed position increment for a bounded discrete trajectory.
 * Speed and acceleration are hard invariants. A target reversal may therefore
 * require temporary motion away from the new target while the prior velocity
 * is brought through zero. */
int32_t head_control_trajectory_step(int32_t position, int32_t previous_step,
                                     int32_t target, int32_t max_step_ticks_per_control_cycle,
                                     int32_t max_acceleration_ticks_per_control_cycle_squared);
void head_control_tick(struct head_runtime *runtime,
                       const struct head_calibration *calibration,
                       uint32_t now_ms);
int32_t head_control_target_tick(const struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t servo_index);
uint32_t head_control_present_current_branch_ma(
    const struct head_runtime *runtime,
    const struct head_calibration *calibration,
    uint8_t branch_index);

#endif
