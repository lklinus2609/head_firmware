#ifndef HEAD_STATE_MACHINE_H_
#define HEAD_STATE_MACHINE_H_

#include "head.h"

bool head_state_can_prepare_motion(const struct head_runtime *runtime);
void head_state_init(struct head_runtime *runtime);
bool head_state_request_home(struct head_runtime *runtime,
                             const struct head_calibration *calibration);
/* Homing variant for a joint with no mechanical stop: drive each servo to the
 * nearest encoder zero and take that as the reference. */
bool head_state_request_zero_home(struct head_runtime *runtime,
                                  const struct head_calibration *calibration);
bool head_state_request_maintenance_calibration(struct head_runtime *runtime,
                                                const struct head_calibration *calibration);
bool head_state_confirm_maintenance_servo(struct head_runtime *runtime,
                                          struct head_calibration *calibration,
                                          uint8_t servo_index);
bool head_state_request_enable(struct head_runtime *runtime);
/* 0, -EPERM (wrong state, profile invalid, or motion preparation not
 * allowed) or -ERANGE (an active servo is outside its profile range). */
int head_state_request_enable_at_present_pose(struct head_runtime *runtime,
                                              const struct head_calibration *calibration);
bool head_state_request_proprioception(struct head_runtime *runtime,
                                       uint32_t now_ms);
bool head_state_request_routing(struct head_runtime *runtime,
                                const struct head_calibration *calibration);
bool head_state_request_tuning(struct head_runtime *runtime,
                               const struct head_calibration *calibration,
                               uint16_t current_limit_ma);
/* One servo's tuning Goal Current: the requested limit, but never more than
 * that servo's own operating_current_ma. */
uint16_t head_state_tuning_limit_ma(const struct head_runtime *runtime,
                                    const struct head_calibration *calibration,
                                    uint8_t servo_index);
int head_state_tuning_step(struct head_runtime *runtime,
                           const struct head_calibration *calibration,
                           uint8_t servo_index, int8_t step_ticks, uint32_t now_ms);
bool head_state_torque_may_be_on(const struct head_runtime *runtime);
bool head_state_torque_never_commanded_on(const struct head_runtime *runtime);
bool head_state_torque_verified_on(const struct head_runtime *runtime);
void head_state_disable(struct head_runtime *runtime);
void head_state_fault(struct head_runtime *runtime, enum head_fault fault);
void head_state_homing_tick(struct head_runtime *runtime,
                            const struct head_calibration *calibration,
                            uint32_t now_ms);
void head_state_proprioception_tick(struct head_runtime *runtime,
                                    uint32_t now_ms);

#endif
