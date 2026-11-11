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

/* Same rule as src/state_machine.c: the requested tuning limit, but never more
 * than the servo's own operating_current_ma. */
uint16_t head_state_tuning_limit_ma(const struct head_runtime *runtime,
                                    const struct head_calibration *calibration,
                                    uint8_t servo_index)
{
  const int16_t operating_ma = calibration->joints[servo_index].operating_current_ma;
  if (operating_ma <= 0) return 0u;
  return runtime->tuning_current_limit_ma < (uint16_t)operating_ma ?
      runtime->tuning_current_limit_ma : (uint16_t)operating_ma;
}

static uint8_t registers[HEAD_SERVO_COUNT][160];
static uint8_t response[128];
static size_t response_length;
static size_t response_offset;
/* Stray 0xFF bytes put in front of the next status packet. */
static size_t status_junk_byte_count;
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
  if (status_junk_byte_count != 0u) {
    memmove(&response[status_junk_byte_count], response, response_length);
    memset(response, 0xFF, status_junk_byte_count);
    response_length += status_junk_byte_count;
    status_junk_byte_count = 0u;
  }
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

/* Telemetry double: the last Sync Read each branch transmitted, and status
 * bytes queued for head_board_branch_read_available() to deliver. */
static uint8_t sync_read_packet[HEAD_BRANCH_COUNT][32];
static size_t sync_read_length[HEAD_BRANCH_COUNT];
static uint8_t telemetry_feed[HEAD_BRANCH_COUNT][1024];
static size_t telemetry_feed_length[HEAD_BRANCH_COUNT];
static size_t telemetry_feed_offset[HEAD_BRANCH_COUNT];

int head_board_branch_write(uint8_t branch_index, const uint8_t *data, size_t length)
{
  assert(length >= 12u);
  if (data[7] == 0x82u) {
    assert(branch_index < HEAD_BRANCH_COUNT && length <= sizeof(sync_read_packet[0]));
    memcpy(sync_read_packet[branch_index], data, length);
    sync_read_length[branch_index] = length;
    return 0;
  }
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
{
  if (branch_index >= HEAD_BRANCH_COUNT) return 0u;
  size_t count = telemetry_feed_length[branch_index] - telemetry_feed_offset[branch_index];
  if (count > capacity) count = capacity;
  memcpy(data, &telemetry_feed[branch_index][telemetry_feed_offset[branch_index]], count);
  telemetry_feed_offset[branch_index] += count;
  return count;
}

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

static int run_preparation(struct head_runtime *runtime, const struct head_calibration *calibration)
{
  int result = 0;
  unsigned steps = 0u;
  do {
    result = head_dxl_prepare_step(runtime, calibration);
    assert(++steps < 100u);
  } while (result == 0);
  return result;
}

static void test_preparation_failure_is_recorded(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  const uint32_t mask = (1u << 0u) | (1u << 3u);
  make_calibration(&calibration, mask);
  memset(registers, 0, sizeof(registers));
  /* Servo 3 reads 13.0 V in phase 3 (voltage check): -ERANGE at once. */
  prepare_runtime(&runtime, &calibration, HEAD_ENABLED);
  registers[3][144] = 130u;
  assert(run_preparation(&runtime, &calibration) == -ERANGE);
  assert(head_dxl_last_preparation_failure() == (uint8_t)((3u << 5u) | 3u));
  /* Servo 0 reads a position past the encoder range in phase 7: -ERANGE,
   * noted as phase 7, servo 0 (the cursor arithmetic for index 0). */
  memset(registers, 0, sizeof(registers));
  prepare_runtime(&runtime, &calibration, HEAD_ENABLED);
  put_u32(&registers[0][132], HEAD_DXL_POSITION_MAX_TICK + 1);
  assert(run_preparation(&runtime, &calibration) == -ERANGE);
  assert(head_dxl_last_preparation_failure() == (uint8_t)((7u << 5u) | 0u));
  puts("DXL preparation failure note: PASS");
}

static void test_tuning_preparation_caps_goal_current(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  const uint32_t mask = (1u << 2u) | (1u << 7u);
  make_calibration(&calibration, mask);   /* operating_current_ma = 100 */
  memset(registers, 0, sizeof(registers));
  goal_writes = 0u;
  torque_enable_writes = 0u;
  /* Servo 7's own operating current is below the requested limit, so its
   * Goal Current is that (2026-10-20 per-servo caps). */
  calibration.joints[7].operating_current_ma = 30;
  prepare_runtime(&runtime, &calibration, HEAD_TUNING);
  runtime.tuning_current_limit_ma = 35u;

  int result = 0;
  unsigned steps = 0u;
  do {
    result = head_dxl_prepare_step(&runtime, &calibration);
    assert(result >= 0);
    assert(++steps < 100u);
  } while (result == 0);
  assert(result == 1);
  assert(torque_enable_writes == 2u);
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((mask & (1u << index)) == 0u) continue;
    /* Goal Current(102) is the tuning limit, and the goal is the present
     * position, so torque-on holds the pose without moving it. */
    const uint8_t expected_ma = index == 7u ? 30u : 35u;
    assert(registers[index][102] == expected_ma && registers[index][103] == 0u);
    assert(runtime.servos[index].goal_tick == runtime.servos[index].present_tick);
    assert(registers[index][98] == DXL_BUS_WATCHDOG_200_MS);
  }
  puts("DXL tuning preparation: PASS");
}

/* Queue one Sync Read status reply whose Present Position is the servo ID and
 * whose Present PWM is -100 x the ID (checks the signed read). Every other
 * telemetry byte is zero, and no value forms FF FF FD, so no byte stuffing is
 * needed. */
static void queue_telemetry_reply(uint8_t branch_index, uint8_t servo_id)
{
  uint8_t *packet = &telemetry_feed[branch_index][telemetry_feed_length[branch_index]];
  const uint16_t packet_length = DXL_TELEMETRY_LENGTH + 4u;
  assert(telemetry_feed_length[branch_index] + 7u + packet_length <=
         sizeof(telemetry_feed[0]));
  memset(packet, 0, 7u + packet_length);
  packet[0] = 0xFFu; packet[1] = 0xFFu; packet[2] = 0xFDu; packet[3] = 0u;
  packet[4] = servo_id;
  packet[5] = (uint8_t)packet_length;
  packet[6] = (uint8_t)(packet_length >> 8u);
  packet[7] = 0x55u;
  packet[8] = 0u;                                /* status error byte */
  const uint16_t present_pwm = (uint16_t)(-100 * (int16_t)servo_id);
  packet[9u + 54u] = (uint8_t)present_pwm;       /* Present PWM (124) */
  packet[9u + 55u] = (uint8_t)(present_pwm >> 8u);
  put_u32(&packet[9u + 62u], servo_id);          /* Present Position (132) */
  const size_t crc_at = 7u + packet_length - 2u;
  const uint16_t crc = test_crc16(packet, crc_at);
  packet[crc_at] = (uint8_t)crc;
  packet[crc_at + 1u] = (uint8_t)(crc >> 8u);
  telemetry_feed_length[branch_index] += 7u + packet_length;
}

static void test_telemetry_follows_profile_branch(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  /* IDs 15-19 on J2, 5-9 on J4 and a lone ID 12 on J3; nothing on J1. */
  make_calibration(&calibration, 0x000FFFE0u & ~((1u << 10u) | (1u << 11u) |
                                                (1u << 13u) | (1u << 14u)));
  for (uint8_t index = 5u; index < 10u; ++index) {
    calibration.joints[index].branch_index = 3u;
    calibration.joints[index + 10u].branch_index = 1u;
  }
  calibration.joints[12].branch_index = 2u;
  memset(&runtime, 0, sizeof(runtime));
  memset(sync_read_length, 0, sizeof(sync_read_length));
  memset(telemetry_feed_length, 0, sizeof(telemetry_feed_length));
  memset(telemetry_feed_offset, 0, sizeof(telemetry_feed_offset));
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) runtime.servos[index].online = true;
  struct head_dxl_bus_stats stats_before[HEAD_BRANCH_COUNT];
  uint32_t missed_before[HEAD_SERVO_COUNT];
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    head_dxl_bus_stats(branch_index, &stats_before[branch_index]);
  }
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    missed_before[index] = head_dxl_missed_reply_count(index);
  }

  head_dxl_telemetry_tick(&runtime, &calibration, 100u, true);
  assert(sync_read_length[0] == 0u && !runtime.branches[0].telemetry_active);
  static const uint8_t expected_ids[HEAD_BRANCH_COUNT][HEAD_SERVOS_PER_BRANCH] = {
    { 0u }, { 15u, 16u, 17u, 18u, 19u }, { 12u }, { 5u, 6u, 7u, 8u, 9u } };
  static const uint8_t expected_count[HEAD_BRANCH_COUNT] = { 0u, 5u, 1u, 5u };
  for (uint8_t branch_index = 1u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    assert(sync_read_length[branch_index] == 12u + expected_count[branch_index] + 2u);
    assert(memcmp(&sync_read_packet[branch_index][12], expected_ids[branch_index],
                  expected_count[branch_index]) == 0);
    assert(runtime.branches[branch_index].telemetry_active);
    assert(runtime.branches[branch_index].telemetry_expected_mask ==
           (uint8_t)((1u << expected_count[branch_index]) - 1u));
  }

  /* J2 loses ID 17; J4 answers in full, after a burst of line noise that
   * includes a false 0xFF; J3 stays silent. */
  memset(telemetry_feed[3], 0x55, 100u);
  telemetry_feed[3][60] = 0xFFu;
  telemetry_feed_length[3] = 100u;
  queue_telemetry_reply(1u, 15u); queue_telemetry_reply(1u, 16u);
  queue_telemetry_reply(1u, 18u); queue_telemetry_reply(1u, 19u);
  for (uint8_t servo_id = 5u; servo_id < 10u; ++servo_id) queue_telemetry_reply(3u, servo_id);
  head_dxl_telemetry_tick(&runtime, &calibration, 101u, false);
  assert(runtime.branches[1].telemetry_received_mask == 0x1Bu);
  assert(runtime.branches[1].telemetry_active);
  assert(runtime.branches[3].telemetry_received_mask == 0x1Fu);
  assert(!runtime.branches[3].telemetry_active);
  assert(runtime.branches[1].protocol_errors == 0u && runtime.branches[3].protocol_errors == 0u);
  /* Noise is skipped in a few moves, not one buffer shift per byte. */
  assert(head_dxl_trace.bad_used_count == 0u);
  for (uint8_t servo_id = 5u; servo_id < 10u; ++servo_id) {
    assert(runtime.servos[servo_id].present_tick == servo_id);
    assert(runtime.servos[servo_id].present_pwm_raw == -100 * (int16_t)servo_id);
    assert(runtime.servos[servo_id + 10u].present_tick ==
           (servo_id + 10u == 17u ? 0 : servo_id + 10));
  }

  head_dxl_telemetry_tick(&runtime, &calibration, 100u + HEAD_TELEMETRY_RESPONSE_TIMEOUT_MS, false);
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    const bool is_missing = index == 17u || index == 12u;
    const bool is_active = (calibration.active_servo_mask & (1u << index)) != 0u;
    assert(runtime.servos[index].online == (!is_active || !is_missing));
  }
  assert(runtime.branches[1].telemetry_timeouts == 1u);
  assert(runtime.branches[2].telemetry_timeouts == 1u);
  assert(runtime.branches[3].telemetry_timeouts == 0u);

  /* Bus statistics separate a silent branch (J3) from one that lost a member
   * (J2), and count the 100 noise bytes J4 skipped before its first header. */
  struct head_dxl_bus_stats stats[HEAD_BRANCH_COUNT];
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    head_dxl_bus_stats(branch_index, &stats[branch_index]);
  }
  assert(stats[0].request_count == stats_before[0].request_count);
  assert(stats[1].request_count - stats_before[1].request_count == 1u);
  assert(stats[1].partial_timeout_count - stats_before[1].partial_timeout_count == 1u);
  assert(stats[1].silent_timeout_count == stats_before[1].silent_timeout_count);
  assert(stats[2].silent_timeout_count - stats_before[2].silent_timeout_count == 1u);
  assert(stats[2].partial_timeout_count == stats_before[2].partial_timeout_count);
  assert(stats[3].complete_count - stats_before[3].complete_count == 1u);
  assert(stats[3].resync_byte_count - stats_before[3].resync_byte_count == 100u);
  assert(stats[3].bad_packet_count == stats_before[3].bad_packet_count);
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    const uint32_t missed = head_dxl_missed_reply_count(index) - missed_before[index];
    assert(missed == ((index == 17u || index == 12u) ? 1u : 0u));
  }
  puts("DXL telemetry follows profile branch: PASS");
}

static void test_status_header_search(void)
{
  /* A stray byte before a register reply (late data, a glitch at the
   * direction switch) must not fail the read; 0xFF is the hardest case. */
  uint8_t value = 0u;
  registers[2][9] = 0x5Au;
  status_junk_byte_count = 1u;
  assert(head_dxl_debug_read(0u, 2u, 9u, 1u, &value) == 0 && value == 0x5Au);
  puts("DXL status header search: PASS");
}

static void test_telemetry_survives_damaged_member(void)
{
  struct head_calibration calibration;
  struct head_runtime runtime;
  make_calibration(&calibration, 0x1Fu);                  /* IDs 0-4 on J1 */
  memset(&runtime, 0, sizeof(runtime));
  memset(telemetry_feed_length, 0, sizeof(telemetry_feed_length));
  memset(telemetry_feed_offset, 0, sizeof(telemetry_feed_offset));
  struct head_dxl_bus_stats before;
  struct head_dxl_bus_stats after;
  head_dxl_bus_stats(0u, &before);
  head_dxl_telemetry_tick(&runtime, &calibration, 200u, true);
  assert(runtime.branches[0].telemetry_active);

  /* Member 1 has one data byte changed; member 3 has one byte missing. */
  for (uint8_t servo_id = 0u; servo_id < 5u; ++servo_id) {
    const size_t start = telemetry_feed_length[0];
    queue_telemetry_reply(0u, servo_id);
    if (servo_id == 1u) telemetry_feed[0][start + 20u] ^= 0x01u;
    if (servo_id == 3u) {
      memmove(&telemetry_feed[0][start + 20u], &telemetry_feed[0][start + 21u],
              telemetry_feed_length[0] - start - 21u);
      --telemetry_feed_length[0];
    }
  }
  head_dxl_telemetry_tick(&runtime, &calibration, 201u, false);
  /* Only the damaged members are lost: dropping one byte and searching again
   * keeps member 4, which skipping the whole claimed packet 3 would cut. */
  assert(runtime.branches[0].telemetry_received_mask == 0x15u);
  head_dxl_bus_stats(0u, &after);
  assert(after.bad_packet_count - before.bad_packet_count == 2u);
  assert(after.bad_in_place_count - before.bad_in_place_count == 1u);
  assert(after.bad_shifted_count - before.bad_shifted_count == 1u);
  puts("DXL telemetry survives a damaged member: PASS");
}

int main(void)
{
  test_alert_policy();
  test_incremental_preparation();
  test_homing_prepares_only_selected_servo();
  test_telemetry_follows_profile_branch();
  test_telemetry_survives_damaged_member();
  test_status_header_search();
  test_tuning_preparation_caps_goal_current();
  test_preparation_failure_is_recorded();
  return 0;
}
