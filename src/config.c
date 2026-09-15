#include <errno.h>
#include <string.h>

#if defined(CONFIG_NVS) && CONFIG_NVS && defined(CONFIG_FLASH_MAP) && CONFIG_FLASH_MAP
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#endif

#include "config.h"
#include "protocol.h"

#define HEAD_NVS_CALIBRATION_A_ID 1u
#define HEAD_NVS_CALIBRATION_B_ID 2u
#define HEAD_CONFIG_RECORD_MAGIC 0x48434647u /* "HCFG" */
#define HEAD_CONFIG_RECORD_COMMIT 0x434F4D54u /* "COMT" */
#define HEAD_CONFIG_RECORD_HEADER_SIZE 28u
#define HEAD_CONFIG_RECORD_SIZE \
  (HEAD_CONFIG_RECORD_HEADER_SIZE + HEAD_CONFIG_STORAGE_PAYLOAD_SIZE)

static uint32_t storage_generation;
static uint16_t storage_active_id;

static bool finite_float(float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7F800000u) != 0x7F800000u;
}

static uint16_t get_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
}

static uint32_t get_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
         ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
}

static void put_u16_le(uint8_t *data, size_t *byte_offset, uint16_t value)
{
  data[(*byte_offset)++] = (uint8_t)value;
  data[(*byte_offset)++] = (uint8_t)(value >> 8u);
}

static void put_u32_le(uint8_t *data, size_t *byte_offset, uint32_t value)
{
  for (uint8_t byte = 0u; byte < 4u; ++byte) {
    data[(*byte_offset)++] = (uint8_t)(value >> (8u * byte));
  }
}

static void put_f32_le(uint8_t *data, size_t *byte_offset, float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  put_u32_le(data, byte_offset, bits);
}

static float get_f32_le(const uint8_t *data)
{
  const uint32_t bits = get_u32_le(data);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

#if defined(CONFIG_NVS) && CONFIG_NVS && defined(CONFIG_FLASH_MAP) && CONFIG_FLASH_MAP
#define HEAD_STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(storage)
#define HEAD_FLASH_NODE DT_MTD_FROM_FIXED_PARTITION(HEAD_STORAGE_NODE)
static struct nvs_fs nvs;
static bool nvs_ready;
static uint8_t storage_record[HEAD_CONFIG_RECORD_SIZE];
static struct head_calibration storage_candidate_a;
static struct head_calibration storage_candidate_b;

static int config_nvs_init(void)
{
  const struct device *flash = DEVICE_DT_GET(HEAD_FLASH_NODE);
  struct flash_pages_info page;
  int result;

  if (nvs_ready) return 0;
  if (!device_is_ready(flash)) return -ENODEV;
  nvs.flash_device = flash;
  nvs.offset = DT_REG_ADDR(HEAD_STORAGE_NODE);
  result = flash_get_page_info_by_offs(flash, nvs.offset, &page);
  if (result != 0) return result;
  nvs.sector_size = page.size;
  nvs.sector_count = 3u;
  result = nvs_mount(&nvs);
  if (result == 0) nvs_ready = true;
  return result;
}
#else
static int config_nvs_init(void) { return -ENOTSUP; }
#endif

/*
 * The board-specific NVS partition is introduced only after the flash layout
 * has been reserved in production.  Until then, defaults are intentionally
 * invalid for homing, so an uncommissioned controller can never enable.
 */
void head_config_default(struct head_calibration *calibration)
{
  memset(calibration, 0, sizeof(*calibration));
  calibration->version = 4u;
  calibration->expected_servo_count = HEAD_SERVO_COUNT;
  calibration->active_servo_mask = 0u; /* Invalid until a tool provisions it. */
  calibration->thermal_start_c = 45u;
  calibration->thermal_full_c = 60u;
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    struct head_joint_config *joint = &calibration->joints[servo_index];
    joint->branch_index = servo_index / HEAD_SERVOS_PER_BRANCH;
    joint->servo_id = servo_index;
    joint->min_tick = 0;
    joint->max_tick = 4095;
    joint->home_tick = 2048;
    joint->max_position_fraction_per_control_cycle = 0.005f;
    joint->max_acceleration_fraction_per_control_cycle_squared = 0.001f;
    joint->homing_timeout_ms = 0u; /* invalid until commissioned */
  }
  head_config_finalize(calibration);
}

void head_config_finalize(struct head_calibration *calibration)
{
  calibration->crc32 = head_protocol_crc32((const uint8_t *)calibration,
                                  offsetof(struct head_calibration, crc32));
}

int head_config_serialize(const struct head_calibration *calibration,
                          uint8_t *output, size_t capacity)
{
  size_t byte_offset = 0u;
  if (calibration == NULL || output == NULL ||
      capacity < HEAD_CONFIG_STORAGE_PAYLOAD_SIZE ||
      !head_config_validate(calibration)) return -EINVAL;

  put_u32_le(output, &byte_offset, calibration->version);
  output[byte_offset++] = calibration->expected_servo_count;
  output[byte_offset++] = calibration->allow_partial_inventory;
  output[byte_offset++] = calibration->thermal_start_c;
  output[byte_offset++] = calibration->thermal_full_c;
  put_u32_le(output, &byte_offset, calibration->active_servo_mask);
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    const struct head_joint_config *joint = &calibration->joints[index];
    output[byte_offset++] = joint->branch_index;
    output[byte_offset++] = joint->servo_id;
    put_u32_le(output, &byte_offset, (uint32_t)joint->min_tick);
    put_u32_le(output, &byte_offset, (uint32_t)joint->max_tick);
    put_u32_le(output, &byte_offset, (uint32_t)joint->home_tick);
    put_f32_le(output, &byte_offset, joint->max_position_fraction_per_control_cycle);
    put_f32_le(output, &byte_offset, joint->max_acceleration_fraction_per_control_cycle_squared);
    put_f32_le(output, &byte_offset, joint->homing_direction);
    put_u32_le(output, &byte_offset, (uint32_t)joint->reserved_homing_start_tick);
    put_u32_le(output, &byte_offset, (uint32_t)joint->homing_max_travel_ticks);
    put_u32_le(output, &byte_offset, joint->homing_timeout_ms);
    put_u16_le(output, &byte_offset, (uint16_t)joint->operating_current_ma);
    put_u16_le(output, &byte_offset, (uint16_t)joint->homing_current_ma);
    put_u32_le(output, &byte_offset, (uint32_t)joint->homing_following_error_ticks);
    put_u16_le(output, &byte_offset, joint->homing_persistence_ms);
    put_u16_le(output, &byte_offset, (uint16_t)joint->homing_current_limit_ma);
    put_u16_le(output, &byte_offset, joint->homing_speed_ticks_per_second);
    put_u32_le(output, &byte_offset, (uint32_t)joint->homing_backoff_ticks);
  }
  return byte_offset == HEAD_CONFIG_STORAGE_PAYLOAD_SIZE ? (int)byte_offset : -EOVERFLOW;
}

bool head_config_deserialize(const uint8_t *input, size_t length,
                             struct head_calibration *calibration)
{
  size_t byte_offset = 0u;
  if (input == NULL || calibration == NULL ||
      length != HEAD_CONFIG_STORAGE_PAYLOAD_SIZE) return false;
  memset(calibration, 0, sizeof(*calibration));
  calibration->version = get_u32_le(&input[byte_offset]); byte_offset += 4u;
  calibration->expected_servo_count = input[byte_offset++];
  calibration->allow_partial_inventory = input[byte_offset++];
  calibration->thermal_start_c = input[byte_offset++];
  calibration->thermal_full_c = input[byte_offset++];
  calibration->active_servo_mask = get_u32_le(&input[byte_offset]); byte_offset += 4u;
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    struct head_joint_config *joint = &calibration->joints[index];
    joint->branch_index = input[byte_offset++];
    joint->servo_id = input[byte_offset++];
    joint->min_tick = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->max_tick = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->home_tick = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->max_position_fraction_per_control_cycle = get_f32_le(&input[byte_offset]); byte_offset += 4u;
    joint->max_acceleration_fraction_per_control_cycle_squared = get_f32_le(&input[byte_offset]); byte_offset += 4u;
    joint->homing_direction = get_f32_le(&input[byte_offset]); byte_offset += 4u;
    joint->reserved_homing_start_tick = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->homing_max_travel_ticks = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->homing_timeout_ms = get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->operating_current_ma = (int16_t)get_u16_le(&input[byte_offset]); byte_offset += 2u;
    joint->homing_current_ma = (int16_t)get_u16_le(&input[byte_offset]); byte_offset += 2u;
    joint->homing_following_error_ticks = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
    joint->homing_persistence_ms = get_u16_le(&input[byte_offset]); byte_offset += 2u;
    joint->homing_current_limit_ma = (int16_t)get_u16_le(&input[byte_offset]); byte_offset += 2u;
    joint->homing_speed_ticks_per_second = get_u16_le(&input[byte_offset]); byte_offset += 2u;
    joint->homing_backoff_ticks = (int32_t)get_u32_le(&input[byte_offset]); byte_offset += 4u;
  }
  if (byte_offset != length) return false;
  head_config_finalize(calibration);
  return head_config_validate(calibration);
}

uint32_t head_config_generation(void) { return storage_generation; }

uint32_t head_config_operating_current_branch_ma(
    const struct head_calibration *calibration, uint8_t branch_index)
{
  uint32_t total_current_ma = 0u;
  if (calibration == NULL || branch_index >= HEAD_BRANCH_COUNT) return UINT32_MAX;
  const uint8_t first_servo = branch_index * HEAD_SERVOS_PER_BRANCH;
  const uint8_t end_servo = first_servo + HEAD_SERVOS_PER_BRANCH;
  for (uint8_t servo_index = first_servo; servo_index < end_servo; ++servo_index) {
    if ((calibration->active_servo_mask & (1u << servo_index)) == 0u) continue;
    const int16_t current_ma = calibration->joints[servo_index].operating_current_ma;
    if (current_ma > 0) total_current_ma += (uint16_t)current_ma;
  }
  return total_current_ma;
}

bool head_config_validate(const struct head_calibration *calibration)
{
  uint32_t crc;

  uint8_t active_count = 0u;

  if (calibration->version != 4u ||
      calibration->expected_servo_count == 0u ||
      calibration->expected_servo_count > HEAD_SERVO_COUNT ||
      calibration->thermal_start_c >= calibration->thermal_full_c ||
      calibration->active_servo_mask == 0u ||
      (calibration->active_servo_mask & ~0x000FFFFFu) != 0u) {
    return false;
  }
  crc = head_protocol_crc32((const uint8_t *)calibration,
                   offsetof(struct head_calibration, crc32));
  if (crc != calibration->crc32) {
    return false;
  }
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    const struct head_joint_config *joint = &calibration->joints[servo_index];
    if ((calibration->active_servo_mask & (1u << servo_index)) == 0u) continue;
    ++active_count;
    if (joint->branch_index != servo_index / HEAD_SERVOS_PER_BRANCH || joint->servo_id != servo_index ||
        joint->min_tick < HEAD_DXL_POSITION_MIN_TICK ||
        joint->max_tick > HEAD_DXL_POSITION_MAX_TICK ||
        joint->min_tick >= joint->max_tick ||
        joint->home_tick < joint->min_tick || joint->home_tick > joint->max_tick ||
        !finite_float(joint->max_position_fraction_per_control_cycle) ||
        joint->max_position_fraction_per_control_cycle <= 0.0f ||
        joint->max_position_fraction_per_control_cycle > 1.0f ||
        !finite_float(joint->max_acceleration_fraction_per_control_cycle_squared) ||
        joint->max_acceleration_fraction_per_control_cycle_squared <= 0.0f ||
        joint->max_acceleration_fraction_per_control_cycle_squared >
            joint->max_position_fraction_per_control_cycle ||
        !finite_float(joint->homing_direction) ||
        joint->homing_direction == 0.0f ||
        joint->reserved_homing_start_tick != 0 ||
        joint->homing_max_travel_ticks <= 0 ||
        joint->homing_max_travel_ticks > HEAD_DXL_POSITION_MAX_TICK - HEAD_DXL_POSITION_MIN_TICK ||
        joint->homing_timeout_ms == 0u || joint->operating_current_ma <= 0 ||
        joint->operating_current_ma > 910 || joint->homing_current_ma <= 0 ||
        joint->homing_current_ma > joint->operating_current_ma ||
        joint->homing_following_error_ticks < 0 ||
        joint->homing_persistence_ms == 0u ||
        joint->homing_current_limit_ma < joint->homing_current_ma ||
        joint->homing_current_limit_ma > joint->operating_current_ma ||
        joint->homing_speed_ticks_per_second == 0u ||
        joint->homing_speed_ticks_per_second > 50000u ||
        joint->homing_backoff_ticks <= 0 ||
        joint->homing_backoff_ticks > joint->homing_max_travel_ticks) {
      return false;
    }
  }
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (head_config_operating_current_branch_ma(calibration, branch_index) >
        HEAD_BRANCH_CURRENT_BUDGET_MA) {
      return false;
    }
  }
  return active_count == calibration->expected_servo_count &&
         (calibration->allow_partial_inventory || active_count == HEAD_SERVO_COUNT);
}

#if defined(CONFIG_NVS) && CONFIG_NVS && defined(CONFIG_FLASH_MAP) && CONFIG_FLASH_MAP
static bool storage_record_decode(const uint8_t *record, size_t length,
                                  struct head_calibration *calibration,
                                  uint32_t *generation)
{
  if (length != HEAD_CONFIG_RECORD_SIZE ||
      get_u32_le(&record[0]) != HEAD_CONFIG_RECORD_MAGIC ||
      get_u16_le(&record[4]) != HEAD_CONFIG_STORAGE_SCHEMA ||
      get_u16_le(&record[6]) != HEAD_CONFIG_RECORD_HEADER_SIZE ||
      get_u32_le(&record[8]) != HEAD_CONFIG_STORAGE_PAYLOAD_SIZE ||
      get_u32_le(&record[16]) != HEAD_CONFIG_HARDWARE_ID ||
      get_u32_le(&record[24]) != HEAD_CONFIG_RECORD_COMMIT ||
      get_u32_le(&record[20]) !=
          head_protocol_crc32(&record[HEAD_CONFIG_RECORD_HEADER_SIZE],
                     HEAD_CONFIG_STORAGE_PAYLOAD_SIZE) ||
      !head_config_deserialize(&record[HEAD_CONFIG_RECORD_HEADER_SIZE],
                               HEAD_CONFIG_STORAGE_PAYLOAD_SIZE, calibration)) {
    return false;
  }
  *generation = get_u32_le(&record[12]);
  return true;
}
#endif

int head_config_load(struct head_calibration *calibration)
{
  int result;
  head_config_default(calibration);
  storage_generation = 0u;
  storage_active_id = 0u;
  result = config_nvs_init();
  if (result != 0) return result;
#if defined(CONFIG_NVS) && CONFIG_NVS && defined(CONFIG_FLASH_MAP) && CONFIG_FLASH_MAP
  uint32_t generation_a = 0u;
  uint32_t generation_b = 0u;
  result = nvs_read(&nvs, HEAD_NVS_CALIBRATION_A_ID,
                storage_record, sizeof(storage_record));
  const bool valid_a = result == sizeof(storage_record) &&
                       storage_record_decode(storage_record, sizeof(storage_record),
                                             &storage_candidate_a, &generation_a);
  result = nvs_read(&nvs, HEAD_NVS_CALIBRATION_B_ID,
                storage_record, sizeof(storage_record));
  const bool valid_b = result == sizeof(storage_record) &&
                       storage_record_decode(storage_record, sizeof(storage_record),
                                             &storage_candidate_b, &generation_b);
  if (!valid_a && !valid_b) {
    return -ENOENT; /* Raw ABI-dependent records are intentionally not migrated. */
  }
  if (valid_b && (!valid_a || (int32_t)(generation_b - generation_a) > 0)) {
    *calibration = storage_candidate_b;
    storage_generation = generation_b;
    storage_active_id = HEAD_NVS_CALIBRATION_B_ID;
  } else {
    *calibration = storage_candidate_a;
    storage_generation = generation_a;
    storage_active_id = HEAD_NVS_CALIBRATION_A_ID;
  }
  return 0;
#else
  return -ENOTSUP;
#endif
}

int head_config_save(const struct head_calibration *calibration)
{
  int result;
  if (!head_config_validate(calibration)) return -EINVAL;
  result = config_nvs_init();
  if (result != 0) return result;
#if defined(CONFIG_NVS) && CONFIG_NVS && defined(CONFIG_FLASH_MAP) && CONFIG_FLASH_MAP
  const uint32_t next_generation = storage_generation + 1u;
  const uint16_t next_id = storage_active_id == HEAD_NVS_CALIBRATION_A_ID ?
                           HEAD_NVS_CALIBRATION_B_ID : HEAD_NVS_CALIBRATION_A_ID;
  size_t byte_offset = 0u;
  memset(storage_record, 0, sizeof(storage_record));
  put_u32_le(storage_record, &byte_offset, HEAD_CONFIG_RECORD_MAGIC);
  put_u16_le(storage_record, &byte_offset, HEAD_CONFIG_STORAGE_SCHEMA);
  put_u16_le(storage_record, &byte_offset, HEAD_CONFIG_RECORD_HEADER_SIZE);
  put_u32_le(storage_record, &byte_offset, HEAD_CONFIG_STORAGE_PAYLOAD_SIZE);
  put_u32_le(storage_record, &byte_offset, next_generation);
  put_u32_le(storage_record, &byte_offset, HEAD_CONFIG_HARDWARE_ID);
  put_u32_le(storage_record, &byte_offset, 0u); /* Payload CRC is filled after encoding. */
  put_u32_le(storage_record, &byte_offset, HEAD_CONFIG_RECORD_COMMIT);
  result = head_config_serialize(calibration,
                             &storage_record[HEAD_CONFIG_RECORD_HEADER_SIZE],
                             HEAD_CONFIG_STORAGE_PAYLOAD_SIZE);
  if (result != HEAD_CONFIG_STORAGE_PAYLOAD_SIZE) return result;
  const uint32_t payload_crc = head_protocol_crc32(
      &storage_record[HEAD_CONFIG_RECORD_HEADER_SIZE],
      HEAD_CONFIG_STORAGE_PAYLOAD_SIZE);
  size_t crc_at = 20u;
  put_u32_le(storage_record, &crc_at, payload_crc);
  result = nvs_write(&nvs, next_id, storage_record, sizeof(storage_record));
  if (result == sizeof(storage_record)) {
    storage_generation = next_generation;
    storage_active_id = next_id;
    return 0;
  }
  return result;
#else
  return -ENOTSUP;
#endif
}
