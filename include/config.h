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

#endif
