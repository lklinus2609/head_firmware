#ifndef HEAD_CONFIG_H_
#define HEAD_CONFIG_H_

#include "head.h"

#define HEAD_CONFIG_STORAGE_SCHEMA 1u
#define HEAD_CONFIG_STORAGE_PAYLOAD_SIZE 1132u
#define HEAD_CONFIG_HARDWARE_ID 0x54343131u /* "T411": Teensy 4.1 carrier. */

void head_config_default(struct head_calibration *calibration);
void head_config_finalize(struct head_calibration *calibration);
bool head_config_validate(const struct head_calibration *calibration);
int head_config_load(struct head_calibration *calibration);
int head_config_save(const struct head_calibration *calibration);
int head_config_serialize(const struct head_calibration *calibration,
                          uint8_t *output, size_t capacity);
bool head_config_deserialize(const uint8_t *input, size_t length,
                             struct head_calibration *calibration);
uint32_t head_config_generation(void);

/* Active servos the calibration assigns to one branch, in ascending servo
 * index. The branch comes from each joint's branch_index, not from its ID, so
 * any ID may sit on any branch. Writes at most HEAD_SERVOS_PER_BRANCH indexes
 * and returns how many it wrote; head_config_validate() rejects a profile that
 * assigns more than that to one branch. Inline so dxl.c needs no link-time
 * dependency on the storage code. */
static inline uint8_t head_config_branch_servo_indexes(
    const struct head_calibration *calibration, uint8_t branch_index,
    uint8_t servo_indexes[HEAD_SERVOS_PER_BRANCH])
{
  uint8_t servo_count = 0u;
  for (uint8_t servo_index = 0u;
       servo_index < HEAD_SERVO_COUNT && servo_count < HEAD_SERVOS_PER_BRANCH;
       ++servo_index) {
    if ((calibration->active_servo_mask & (1u << servo_index)) != 0u &&
        calibration->joints[servo_index].branch_index == branch_index) {
      servo_indexes[servo_count++] = servo_index;
    }
  }
  return servo_count;
}

#endif
