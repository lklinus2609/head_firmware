#ifndef HEAD_FAN_H_
#define HEAD_FAN_H_

#include "head.h"

int head_fan_init(void);
int head_fan_tick(const struct head_runtime *runtime,
                  const struct head_calibration *calibration,
                  uint32_t now_ms);
uint16_t head_fan_rpm(void);
bool head_fan_stalled(void);

#endif
