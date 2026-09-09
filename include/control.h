#ifndef HEAD_CONTROL_H_
#define HEAD_CONTROL_H_

#include "head.h"

bool head_control_reference_valid(const struct head_runtime *runtime,
                                  const struct head_calibration *calibration,
                                  uint8_t servo_index);
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

#endif
