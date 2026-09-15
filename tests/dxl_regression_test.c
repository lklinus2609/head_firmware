#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* The runner supplies a minimal Zephyr kernel shim, while this translation
 * unit includes the production DXL implementation itself. */
#include "../src/dxl.c"

/* The telemetry section references the lifecycle fault hook, although this
 * test links only the DXL implementation. */
void head_state_fault(struct head_runtime *runtime, enum head_fault fault)
{
  runtime->fault = fault;
  runtime->state = HEAD_FAULT;
}

static uint8_t registers[HEAD_SERVO_COUNT][160];
static uint8_t response[128];
static size_t response_length;
static size_t response_offset;
static uint8_t read_error;
static unsigned event_number;
static unsigned first_goal_event;
static unsigned first_torque_enable_event;
static unsigned torque_enable_writes;
static unsigned goal_writes;

static uint16_t test_crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0u;
  for (size_t index = 0u; index < length; ++index) {
    crc ^= (uint16_t)data[index] << 8u;
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
      crc = (crc & 0x8000u) != 0u ?
            (uint16_t)((crc << 1u) ^ 0x8005u) : (uint16_t)(crc << 1u);
    }
  }
  return crc;
}

static void make_status(uint8_t servo_id, const uint8_t *data, size_t length,
                        uint8_t error)
{
  assert(length <= 64u);
  response[0] = 0xFFu;
  response[1] = 0xFFu;
  response[2] = 0xFDu;
  response[3] = 0u;
  response[4] = servo_id;
  response[5] = (uint8_t)(length + 4u);
  response[6] = (uint8_t)((length + 4u) >> 8u);
  response[7] = 0x55u;
  response[8] = error;
  if (length != 0u) memcpy(&response[9], data, length);
  const size_t crc_at = 9u + length;
  const uint16_t crc = test_crc16(response, crc_at);
  response[crc_at] = (uint8_t)crc;
  response[crc_at + 1u] = (uint8_t)(crc >> 8u);
  response_length = crc_at + 2u;
  response_offset = 0u;
}

static void apply_register(uint8_t servo_id, uint16_t address,
                           const uint8_t *data, uint16_t length)
{
  assert(servo_id < HEAD_SERVO_COUNT);
  assert((size_t)address + length <= sizeof(registers[servo_id]));
  memcpy(&registers[servo_id][address], data, length);
}

static void apply_sync_write(const uint8_t *packet)
{
  const uint16_t address = (uint16_t)packet[8] | ((uint16_t)packet[9] << 8u);
  const uint16_t data_length = (uint16_t)packet[10] |
                               ((uint16_t)packet[11] << 8u);
  const size_t packet_length = (size_t)packet[5] |
                               ((size_t)packet[6] << 8u);
  const size_t end = 7u + packet_length - 2u;
  size_t byte_offset = 12u;
  while (byte_offset < end) {
    const uint8_t servo_id = packet[byte_offset++];
    assert(byte_offset + data_length <= end);
    apply_register(servo_id, address, &packet[byte_offset], data_length);
    if (address == 64u && data_length == 1u && packet[byte_offset] == 1u) {
      ++torque_enable_writes;
      if (first_torque_enable_event == 0u) first_torque_enable_event = ++event_number;
      else ++event_number;
    } else if (address == 116u) {
      ++goal_writes;
      if (first_goal_event == 0u) first_goal_event = ++event_number;
      else ++event_number;
    } else {
      ++event_number;
    }
    byte_offset += data_length;
  }
}

int head_board_init(void) { return 0; }
int head_board_branch_set_baud(uint8_t branch_index, uint32_t baudrate)
{ (void)branch_index; (void)baudrate; return 0; }
int head_board_branch_set_tx(uint8_t branch_index, bool tx)
{ (void)branch_index; (void)tx; return 0; }
unsigned int board_rx_restarts[HEAD_BRANCH_COUNT];
int head_board_branch_rx_restart(uint8_t branch_index)
{
  if (branch_index < HEAD_BRANCH_COUNT) ++board_rx_restarts[branch_index];
  return 0;
}
int head_board_release_all(void) { return 0; }
uint32_t head_board_branch_error_flags(uint8_t branch_index)
{ (void)branch_index; return 0u; }
int head_board_branch_uart_tx_meter_test(uint8_t branch_index, uint32_t duration_ms)
{ (void)branch_index; (void)duration_ms; return 0; }
int head_board_branch_uart_rx_line_test(uint8_t branch_index, uint32_t duration_ms,
                                        uint32_t *bytes, uint32_t *errors)
{ (void)branch_index; (void)duration_ms; *bytes = 0u; *errors = 0u; return 0; }

int head_board_branch_write_async(uint8_t branch_index, const uint8_t *data,
                                  size_t length)
{
  (void)branch_index;
  assert(length >= 10u);
  if (data[7] == 0x83u) {
    apply_sync_write(data);
  } else {
    assert(data[7] == 0x03u);
    const uint16_t address = (uint16_t)data[8] | ((uint16_t)data[9] << 8u);
    const uint16_t data_length = (uint16_t)data[5] |
                                 ((uint16_t)data[6] << 8u);
    assert(data_length >= 5u);
    const uint16_t value_length = data_length - 5u;
    if (data[4] == DXL_BROADCAST_ID) {
      for (uint8_t servo_id = 0u; servo_id < HEAD_SERVO_COUNT; ++servo_id) {
        apply_register(servo_id, address, &data[10], value_length);
      }
    } else {
      apply_register(data[4], address, &data[10], value_length);
    }
    ++event_number;
  }
  return 0;
}

int head_board_branch_write_wait(uint8_t branch_index, uint32_t timeout_us)
{ (void)branch_index; (void)timeout_us; return 0; }

int head_board_branch_write(uint8_t branch_index, const uint8_t *data, size_t length)
{
  (void)branch_index;
  assert(length >= 12u);
  assert(data[7] == 0x02u || data[7] == 0x03u);
  const uint8_t servo_id = data[4];
  const uint16_t address = (uint16_t)data[8] | ((uint16_t)data[9] << 8u);
  if (data[7] == 0x02u) {
    const uint16_t value_length = (uint16_t)data[10] |
                                  ((uint16_t)data[11] << 8u);
    assert(servo_id < HEAD_SERVO_COUNT);
    make_status(servo_id, &registers[servo_id][address], value_length, read_error);
    read_error = 0u;
  } else {
    const uint16_t packet_length = (uint16_t)data[5] |
                                   ((uint16_t)data[6] << 8u);
    const uint16_t value_length = packet_length - 5u;
    apply_register(servo_id, address, &data[10], value_length);
    if (address == 64u && value_length == 1u && data[10] == 1u) {
      ++torque_enable_writes;
      if (first_torque_enable_event == 0u) first_torque_enable_event = ++event_number;
      else ++event_number;
    } else if (address == 116u) {
      ++goal_writes;
      if (first_goal_event == 0u) first_goal_event = ++event_number;
      else ++event_number;
    } else {
      ++event_number;
    }
    make_status(servo_id, NULL, 0u, 0u);
  }
  return 0;
}

int head_board_branch_read(uint8_t branch_index, uint8_t *data, size_t length,
                           uint32_t timeout_us)
{
  (void)branch_index;
  (void)timeout_us;
  assert(response_offset + length <= response_length);
  memcpy(data, &response[response_offset], length);
  response_offset += length;
  return 0;
}

size_t head_board_branch_read_available(uint8_t branch_index, uint8_t *data,
                                        size_t capacity)
{ (void)branch_index; (void)data; (void)capacity; return 0u; }

int head_board_branch_wait_rx(uint8_t branch_index, uint32_t timeout_us)
{ (void)branch_index; (void)timeout_us; return -ETIMEDOUT; }

void head_board_branch_rx_stats(uint8_t branch_index, uint16_t *disabled,
                                uint16_t *restart_failures, uint16_t *overflows)
{
  (void)branch_index;
  if (disabled != NULL) *disabled = 0u;
  if (restart_failures != NULL) *restart_failures = 0u;
  if (overflows != NULL) *overflows = 0u;
}

int head_board_fan_set_percent(uint8_t percent)
{ (void)percent; return 0; }
uint16_t head_board_fan_rpm(void) { return 0u; }

static void make_calibration(struct head_calibration *calibration,
                             uint32_t mask)
{
  memset(calibration, 0, sizeof(*calibration));
  calibration->active_servo_mask = mask;
  calibration->expected_servo_count = 0u;
  calibration->allow_partial_inventory = 1u;
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    struct head_joint_config *joint = &calibration->joints[index];
    joint->branch_index = index / HEAD_SERVOS_PER_BRANCH;
    joint->servo_id = index;
    joint->min_tick = 0;
    joint->max_tick = 4095;
    joint->home_tick = 2048;
    joint->operating_current_ma = 100;
    joint->homing_current_ma = 50;
    joint->homing_current_limit_ma = 75;
    joint->homing_direction = 1.0f;
    joint->homing_max_travel_ticks = 100;
    joint->homing_timeout_ms = 1000u;
    joint->homing_persistence_ms = 10u;
    joint->homing_speed_ticks_per_second = 500u;
    joint->homing_backoff_ticks = 5;
    if ((mask & (1u << index)) != 0u) ++calibration->expected_servo_count;
  }
}

static void put_u32(uint8_t *data, int32_t value)
{
  const uint32_t bits = (uint32_t)value;
  data[0] = (uint8_t)bits;
  data[1] = (uint8_t)(bits >> 8u);
  data[2] = (uint8_t)(bits >> 16u);
  data[3] = (uint8_t)(bits >> 24u);
}

static void prepare_runtime(struct head_runtime *runtime,
                            const struct head_calibration *calibration,
                            enum head_state target_state)
{
  memset(runtime, 0, sizeof(*runtime));
  runtime->preparation_target_state = target_state;
  runtime->preparation_phase = 0u;
  runtime->homing_index = 3u;
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((calibration->active_servo_mask & (1u << index)) == 0u) continue;
    runtime->servos[index].online = true;
    runtime->servos[index].present_voltage_mv = 8000u;
    runtime->servos[index].present_tick = 1000 + index;
    put_u32(&registers[index][132], runtime->servos[index].present_tick);
    registers[index][64] = 0u;
    registers[index][98] = 3u;
    registers[index][144] = 80u;
    registers[index][145] = 0u;
  }
}

static void test_alert_policy(void)
{
  struct head_calibration calibration;
  make_calibration(&calibration, 1u);
  registers[0][64] = 0u;
  uint8_t torque = 99u;

  read_error = 0x80u;
  assert(dxl_read_register(0u, 0u, 64u, 1u, &torque) == 0);
  assert(torque == 0u);
  assert(head_dxl_take_hardware_alert_mask() == 1u);
  assert(head_dxl_take_hardware_alert_mask() == 0u);

  read_error = 0x01u;
  assert(dxl_read_register(0u, 0u, 64u, 1u, &torque) == -EPROTO);

  registers[0][64] = 1u;
  read_error = 0x80u;
  assert(dxl_read_register(0u, 0u, 64u, 1u, &torque) == -EIO);
  assert(head_dxl_take_hardware_alert_mask() == 1u);

  (void)calibration;
  puts("DXL alert policy: PASS");
}

static void test_incremental_preparation(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  const uint32_t mask = (1u << 0u) | (1u << 3u);
  make_calibration(&calibration, mask);
  memset(registers, 0, sizeof(registers));
  event_number = 0u;
  first_goal_event = 0u;
  first_torque_enable_event = 0u;
  torque_enable_writes = 0u;
  goal_writes = 0u;
  prepare_runtime(&runtime, &calibration, HEAD_ENABLED);

  int result = 0;
  unsigned steps = 0u;
  do {
    result = head_dxl_prepare_step(&runtime, &calibration);
    assert(result >= 0);
    assert(++steps < 100u);
  } while (result == 0);

  assert(result == 1);
  assert(steps > 8u);
  assert(torque_enable_writes == 2u);
  assert(goal_writes == 2u);
  assert(first_goal_event != 0u);
  assert(first_torque_enable_event > first_goal_event);
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((mask & (1u << index)) == 0u) continue;
    assert(runtime.servos[index].goal_tick == runtime.servos[index].present_tick);
    assert(runtime.servos[index].last_feedback_ms == 0u);
    assert(registers[index][64] == 1u);
  }
  assert(runtime.torque_state == HEAD_TORQUE_SHUTDOWN_PENDING);
  puts("DXL incremental preparation: PASS");
}

static void test_homing_prepares_only_selected_servo(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  const uint32_t mask = (1u << 0u) | (1u << 3u);
  make_calibration(&calibration, mask);
  memset(registers, 0, sizeof(registers));
  event_number = 0u;
  first_goal_event = 0u;
  first_torque_enable_event = 0u;
  torque_enable_writes = 0u;
  goal_writes = 0u;
  prepare_runtime(&runtime, &calibration, HEAD_HOMING);

  int result = 0;
  unsigned steps = 0u;
  do {
    result = head_dxl_prepare_step(&runtime, &calibration);
    assert(result >= 0);
    assert(++steps < 100u);
  } while (result == 0);
  assert(result == 1);
  assert(goal_writes == 1u);
  assert(torque_enable_writes == 1u);
  assert(runtime.servos[3].goal_tick == runtime.servos[3].present_tick);
  assert(registers[3][64] == 1u);
  assert(registers[0][64] == 0u);
  puts("DXL homing selection: PASS");
}

int main(void)
{
  test_alert_policy();
  test_incremental_preparation();
  test_homing_prepares_only_selected_servo();
  return 0;
}
