#ifndef HEAD_STATE_MACHINE_H_
#define HEAD_STATE_MACHINE_H_

#include "head.h"

bool head_state_can_prepare_motion(const struct head_runtime *runtime);
void head_state_init(struct head_runtime *runtime);
bool head_state_request_home(struct head_runtime *runtime,
                             const struct head_calibration *calibration);
bool head_state_request_maintenance_calibration(struct head_runtime *runtime,
                                                const struct head_calibration *calibration);
bool head_state_confirm_maintenance_servo(struct head_runtime *runtime,
                                          struct head_calibration *calibration,
                                          uint8_t servo_index);
bool head_state_request_enable(struct head_runtime *runtime);
bool head_state_request_proprioception(struct head_runtime *runtime,
                                       uint32_t now_ms);
bool head_state_request_routing(struct head_runtime *runtime,
                                const struct head_calibration *calibration);
bool head_state_torque_may_be_on(const struct head_runtime *runtime);
bool head_state_torque_verified_on(const struct head_runtime *runtime);
void head_state_disable(struct head_runtime *runtime);
void head_state_fault(struct head_runtime *runtime, enum head_fault fault);
void head_state_homing_tick(struct head_runtime *runtime,
                            const struct head_calibration *calibration,
                            uint32_t now_ms);
void head_state_proprioception_tick(struct head_runtime *runtime,
                                    uint32_t now_ms);

#endif
