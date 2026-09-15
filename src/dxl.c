#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "board.h"
#include "dxl.h"
#include "state_machine.h"

/* RX is deliberately kept per physical UART.  Telemetry polling must never
 * wait for a reply from one branch while preventing another branch from being
 * serviced. */
/* A full five-servo 70..146 Sync Read is 440 bytes before byte stuffing.
 * Stage a complete branch response so a partial packet at the end of one DMA
 * drain can be combined safely with the next. */
#define DXL_TELEMETRY_BRANCH_RX_CAPACITY 640u
static uint8_t telemetry_rx[HEAD_BRANCH_COUNT][DXL_TELEMETRY_BRANCH_RX_CAPACITY];
static size_t telemetry_rx_used[HEAD_BRANCH_COUNT];
static uint32_t hardware_alert_servo_mask;
static uint8_t discovery_reason[HEAD_SERVO_COUNT];
/* Commissioning must be able to talk to a servo that is reporting a Hardware
 * Alert, because the alert may be caused by a limit this firmware itself wrote
 * (a Max Voltage Limit below the actual rail latches Input Voltage Error).
 * Refusing every transfer would make that unrecoverable: the repair write is
 * exactly what the alert blocks. Reads/writes stay refused everywhere else. */
static bool alert_tolerant_configuration;
/* Extra read attempts allowed for the current caller, and a saturating total
 * so the underlying loss rate stays visible instead of being hidden. */
static uint8_t read_retry_attempts;
static uint16_t read_retry_count;
/* Last Goal Position actually put on the wire, so a profile-driven move is not
 * restarted by a redundant rewrite. Only consulted while zero homing. */
static int32_t transmitted_goal_tick[HEAD_SERVO_COUNT];
static bool transmitted_goal_valid;
static uint32_t transmitted_goal_ms;
/* Address and raw Protocol 2.0 error byte of the most recent rejected
 * register transfer, so a failed ensure/read names the register and the
 * servo's own complaint (bits 0-6: 4=data range, 6=data limit, 7=access). */
static uint16_t last_transaction_address;
static uint8_t last_transaction_status;

void head_dxl_last_transaction_error(uint16_t *address, uint8_t *status)
{
  if (address != NULL) *address = last_transaction_address;
  if (status != NULL) *status = last_transaction_status;
}

static void note_transaction_error(uint16_t address, uint8_t status)
{
  last_transaction_address = address;
  last_transaction_status = status;
}

uint8_t head_dxl_discovery_reason(uint8_t servo_index)
{
  return servo_index < HEAD_SERVO_COUNT ? discovery_reason[servo_index] : 0u;
}

uint32_t head_dxl_take_hardware_alert_mask(void)
{
  const uint32_t alert_mask = hardware_alert_servo_mask;
  hardware_alert_servo_mask = 0u;
  return alert_mask;
}

#define DXL_TELEMETRY_START_ADDRESS 70u
#define DXL_TELEMETRY_LENGTH 77u
#define DXL_XC330_T181_MODEL_NUMBER 1210u
#define DXL_OPERATING_MODE_CURRENT_POSITION 5u
#define DXL_BAUD_1M 3u
#define DXL_BUS_WATCHDOG_200_MS 10u
/* Preparation arms the servo's own Bus Watchdog, which stops the motor when no
 * goal update arrives inside its window. Suppressing unchanged Goal Position
 * writes stops a profile being restarted, but suppressing them indefinitely
 * starves that watchdog: it trips, the servo stops, traffic resumes, it drives
 * again -- a 5 Hz stop-go cycle that reads as the joint oscillating on target.
 * Refresh comfortably inside the window instead of never. */
#define DXL_GOAL_REFRESH_MS 100u
#define DXL_MINIMUM_WATCHDOG_FIRMWARE 38u
#define DXL_STARTUP_CONFIGURATION_FIRMWARE 46u
#define DXL_STARTUP_TORQUE_ON_MASK 0x01u
#define DXL_STATUS_RETURN_ALL 2u
/* Overheating, electrical shock and overload (bits 2, 4, 5). This servo
 * rejects bit 0 (Input Voltage Error) with a Data Range Error, so it cannot be
 * part of the servo's autonomous torque-off set. Input voltage is still
 * supervised: Hardware Error Status reports it independently of this register,
 * telemetry faults on any non-zero hardware error, and preparation range-checks
 * Present Input Voltage before enabling torque. */
#define DXL_SHUTDOWN_REQUIRED_MASK 0x34u
#define DXL_SECONDARY_ID_DISABLED 255u
/* Deliberately far longer than a healthy ~400 us turnaround. A reply that only
 * shows up tens of milliseconds late is a delivery problem in the firmware RX
 * path; total silence across this window is not. Only used with torque off. */
#define DXL_DEBUG_PING_WINDOW_US 40000u
/* Control-table addresses below this are EEPROM; at and above it are RAM. */
/* Zero-home trajectory, in control-table units: Profile Velocity steps of
 * 0.229 rev/min and Profile Acceleration steps of 214.577 rev/min^2. 20 and 5
 * are about 4.6 rev/min (~27 deg/s) reached over roughly a quarter second. */
/* Zero means unlimited, so with both at zero the servo runs at its Velocity
 * Limit and is still at full speed when it reaches the goal: the position loop
 * absorbs all of it and rings. A short move never builds that speed, which is
 * why an unprofiled hop looks clean and a revolution-and-a-half does not. These
 * give the trapezoid a deceleration ramp instead. Units are 0.229 rev/min and
 * 214.577 rev/min^2, so this is ~34 rev/min reached over about a tenth of a
 * second -- a full revolution in roughly 1.7 s, inside the homing timeout. */
#define DXL_ZERO_HOME_PROFILE_ACCELERATION 0u
#define DXL_ZERO_HOME_PROFILE_VELOCITY 0u
#define DXL_EEPROM_ADDRESS_END 64u
#define DXL_EEPROM_COMMIT_RETRIES 6u
#define DXL_EEPROM_COMMIT_DELAY_MS 3u
#define DXL_MINIMUM_SAFE_VOLTAGE_MV 6500u
/* XC330-T181 datasheet: Input Voltage 6.5 ~ 12.0 V (recommended 11.1 V).
 * Do not raise this to accommodate an out-of-spec supply. This bounds the
 * measured steady-state supply before torque-on and is deliberately NOT the
 * value written to the servo's own Max Voltage Limit (address 32), which is
 * a per-sample trip that must tolerate regenerative transients. */
#define DXL_MAXIMUM_SAFE_VOLTAGE_MV 12000u

static int dxl_verify_register_u8_all(const struct head_calibration *calibration,
                                      uint16_t address, uint8_t expected);
static int dxl_ensure_register(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                               const uint8_t *expected, uint16_t length);
static int dxl_read_register(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                             uint16_t length, uint8_t *data);
#define DXL_DISCOVERY_READ_RETRIES 2u
#define HEAD_PREPARATION_STEP_RETRIES 3u
static int dxl_write_register_u8(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                                 uint8_t value);
static int dxl_write_register_bytes(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                                    const uint8_t *data, uint16_t length);

static bool servo_active(const struct head_calibration *calibration, uint8_t index)
{
  return index < HEAD_SERVO_COUNT &&
         (calibration->active_servo_mask & (1u << index)) != 0u;
}

static int servo_index_for_id(const struct head_calibration *calibration,
                              uint8_t branch_index, uint8_t servo_id)
{
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if (servo_active(calibration, index) && calibration->joints[index].branch_index == branch_index &&
        calibration->joints[index].servo_id == servo_id) {
      return (int)index;
    }
  }
  return -1;
}

static uint16_t dxl_crc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0u;
  for (size_t servo_index = 0; servo_index < length; ++servo_index) {
    crc ^= (uint16_t)data[servo_index] << 8u;
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1u) ^ 0x8005u)
                             : (uint16_t)(crc << 1u);
    }
  }
  return crc;
}

static size_t dxl_finish(uint8_t *packet, size_t payload_end, size_t capacity)
{
  size_t stuffed_count = 0u;
  if (packet == NULL || payload_end < 8u) return 0u;

  /* Protocol 2.0 stuffing covers Instruction through the final Parameter.
   * Count first so insufficient capacity cannot leave a partial packet. */
  for (size_t servo_index = 9u; servo_index < payload_end; ++servo_index) {
    if (packet[servo_index - 2u] == 0xFFu && packet[servo_index - 1u] == 0xFFu && packet[servo_index] == 0xFDu) {
      ++stuffed_count;
    }
  }
  if (payload_end + stuffed_count + 2u > capacity) return 0u;

  for (size_t servo_index = 9u; servo_index < payload_end; ++servo_index) {
    if (packet[servo_index - 2u] == 0xFFu && packet[servo_index - 1u] == 0xFFu && packet[servo_index] == 0xFDu) {
      memmove(&packet[servo_index + 2u], &packet[servo_index + 1u], payload_end - servo_index - 1u);
      packet[servo_index + 1u] = 0xFDu;
      ++payload_end;
      ++servo_index;
    }
  }

  const uint16_t length = (uint16_t)packet[5] | ((uint16_t)packet[6] << 8u);
  const uint16_t stuffed_length = length + (uint16_t)stuffed_count;
  packet[5] = (uint8_t)stuffed_length;
  packet[6] = (uint8_t)(stuffed_length >> 8u);
  const uint16_t crc = dxl_crc16(packet, payload_end);
  packet[payload_end] = (uint8_t)(crc & 0xFFu);
  packet[payload_end + 1u] = (uint8_t)(crc >> 8u);
  return payload_end + 2u;
}

static size_t dxl_ping(uint8_t servo_id, uint8_t *output, size_t capacity)
{
  if (capacity < 10u) return 0u;
  output[0] = 0xFFu; output[1] = 0xFFu; output[2] = 0xFDu; output[3] = 0u;
  output[4] = servo_id; output[5] = 3u; output[6] = 0u; output[7] = 0x01u;
  return dxl_finish(output, 8u, capacity);
}

static size_t dxl_read(uint8_t servo_id, uint16_t address, uint16_t length,
                       uint8_t *output, size_t capacity)
{
  if (capacity < 14u || length == 0u) return 0u;
  output[0] = 0xFFu; output[1] = 0xFFu; output[2] = 0xFDu; output[3] = 0u;
  output[4] = servo_id; output[5] = 7u; output[6] = 0u; output[7] = 0x02u;
  output[8] = (uint8_t)address; output[9] = (uint8_t)(address >> 8u);
  output[10] = (uint8_t)length; output[11] = (uint8_t)(length >> 8u);
  return dxl_finish(output, 12u, capacity);
}

static size_t dxl_write(uint8_t servo_id, uint16_t address, const uint8_t *data,
                        uint16_t length, uint8_t *output, size_t capacity)
{
  if (data == NULL || length == 0u || capacity < (size_t)length + 12u) return 0u;
  output[0] = 0xFFu; output[1] = 0xFFu; output[2] = 0xFDu; output[3] = 0u;
  output[4] = servo_id; output[5] = (uint8_t)(5u + length);
  output[6] = (uint8_t)((5u + length) >> 8u); output[7] = 0x03u;
  output[8] = (uint8_t)address; output[9] = (uint8_t)(address >> 8u);
  memcpy(&output[10], data, length);
  return dxl_finish(output, 10u + length, capacity);
}

static bool dxl_status_valid(const uint8_t *packet, size_t length, uint8_t servo_id)
{
  uint16_t crc;
  uint16_t received;
  if (length < 11u || packet[0] != 0xFFu || packet[1] != 0xFFu ||
      packet[2] != 0xFDu || packet[3] != 0u || packet[4] != servo_id ||
      packet[7] != 0x55u || length != 7u + (size_t)packet[5] +
      ((size_t)packet[6] << 8u)) return false;
  received = (uint16_t)((uint16_t)packet[length - 2u] |
                        ((uint16_t)packet[length - 1u] << 8u));
  crc = dxl_crc16(packet, length - 2u);
  return crc == received;
}

/* Protocol 2.0 inserts an extra 0xFD after an in-payload FF FF FD sequence.
 * CRC is checked on the transmitted (stuffed) packet; register offsets are
 * interpreted only after removing that inserted byte. */
static size_t dxl_unstuff_status_parameters(const uint8_t *packet, size_t length,
                                            uint8_t *output, size_t capacity)
{
  size_t input = 8u; /* Status error byte is the first parameter. */
  size_t used = 0u;
  const size_t end = length - 2u; /* Exclude CRC. */
  while (input < end && used < capacity) {
    output[used++] = packet[input++];
    if (used >= 3u && output[used - 3u] == 0xFFu && output[used - 2u] == 0xFFu &&
        output[used - 1u] == 0xFDu && input < end && packet[input] == 0xFDu) {
      ++input;
    }
  }
  return input == end ? used : 0u;
}

static size_t dxl_sync_read(uint8_t branch_index, const struct head_calibration *calibration,
                            uint8_t *output, size_t capacity)
{
  size_t byte_offset = 0u;
  uint8_t member_count = 0u;
  if (capacity < 32u || branch_index >= HEAD_BRANCH_COUNT) return 0u;
  output[byte_offset++] = 0xFFu; output[byte_offset++] = 0xFFu; output[byte_offset++] = 0xFDu; output[byte_offset++] = 0u;
  output[byte_offset++] = DXL_BROADCAST_ID; output[byte_offset++] = 0u; output[byte_offset++] = 0u;
  output[byte_offset++] = 0x82u;             /* Sync Read */
  /* One contiguous region includes Hardware Error Status (70) as well as
   * Moving, Moving Status, Current, Position, Voltage, and Temperature.
   * Reading the status-packet Error byte alone is not a hardware diagnostic. */
  output[byte_offset++] = DXL_TELEMETRY_START_ADDRESS & 0xFFu;
  output[byte_offset++] = DXL_TELEMETRY_START_ADDRESS >> 8u;
  output[byte_offset++] = DXL_TELEMETRY_LENGTH & 0xFFu;
  output[byte_offset++] = DXL_TELEMETRY_LENGTH >> 8u;
  for (uint8_t offset = 0; offset < HEAD_SERVOS_PER_BRANCH; ++offset) {
    const uint8_t index = branch_index * HEAD_SERVOS_PER_BRANCH + offset;
    if (servo_active(calibration, index)) {
      output[byte_offset++] = calibration->joints[index].servo_id;
      ++member_count;
    }
  }
  if (member_count == 0u) return 0u;
  const uint16_t packet_length = 3u + 4u + member_count;
  output[5] = (uint8_t)packet_length;
  output[6] = (uint8_t)(packet_length >> 8u);
  return dxl_finish(output, byte_offset, capacity);
}

static size_t dxl_sync_write_torque(uint8_t branch_index,
                                    const struct head_calibration *calibration,
                                    bool enabled, uint8_t *output, size_t capacity)
{
  size_t byte_offset = 0u;
  uint8_t member_count = 0u;
  if (capacity < 48u || branch_index >= HEAD_BRANCH_COUNT) return 0u;
  output[byte_offset++] = 0xFFu; output[byte_offset++] = 0xFFu; output[byte_offset++] = 0xFDu; output[byte_offset++] = 0u;
  output[byte_offset++] = DXL_BROADCAST_ID; output[byte_offset++] = 0u; output[byte_offset++] = 0u;
  output[byte_offset++] = 0x83u;             /* Sync Write */
  output[byte_offset++] = 64u; output[byte_offset++] = 0u; /* Torque Enable */
  output[byte_offset++] = 1u; output[byte_offset++] = 0u;
  for (uint8_t offset = 0; offset < HEAD_SERVOS_PER_BRANCH; ++offset) {
    const uint8_t index = branch_index * HEAD_SERVOS_PER_BRANCH + offset;
    if (!servo_active(calibration, index)) continue;
    output[byte_offset++] = calibration->joints[index].servo_id;
    output[byte_offset++] = enabled ? 1u : 0u;
    ++member_count;
  }
  if (member_count == 0u) return 0u;
  const uint16_t packet_length = 3u + 4u + (uint16_t)member_count * 2u;
  output[5] = (uint8_t)packet_length;
  output[6] = (uint8_t)(packet_length >> 8u);
  return dxl_finish(output, byte_offset, capacity);
}

int head_dxl_init(void) { return head_board_init(); }

int head_dxl_emergency_torque_off(void)
{
  uint8_t packets[HEAD_BRANCH_COUNT][20];
  bool started[HEAD_BRANCH_COUNT] = { false };
  const uint8_t disabled = 0u;
  int result = 0;

  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    const size_t size = dxl_write(DXL_BROADCAST_ID, 64u, &disabled,
                                  sizeof(disabled), packets[branch_index],
                                  sizeof(packets[branch_index]));
    if (size == 0u ||
        head_board_branch_write_async(branch_index, packets[branch_index], size) != 0) {
      result = -EIO;
    } else {
      started[branch_index] = true;
    }
  }
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (started[branch_index] && head_board_branch_write_wait(branch_index, 3000u) != 0) {
      result = -EIO;
    }
  }
  return result;
}

int head_dxl_set_torque_all(const struct head_calibration *calibration, bool enabled)
{
  uint8_t packet[HEAD_BRANCH_COUNT][48];
  bool started[HEAD_BRANCH_COUNT] = { false };
  int result = 0;
  for (uint8_t branch_index = 0; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    const size_t size = dxl_sync_write_torque(branch_index, calibration, enabled,
                                               packet[branch_index], sizeof(packet[branch_index]));
    if (size == 0u) continue;
    if (head_board_branch_write_async(branch_index, packet[branch_index], size) == 0) {
      started[branch_index] = true;
    } else {
      result = -EIO;
    }
  }
  for (uint8_t branch_index = 0; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (started[branch_index] && head_board_branch_write_wait(branch_index, 3000u) != 0) result = -EIO;
  }
  if (result == 0) {
    result = dxl_verify_register_u8_all(calibration, 64u, enabled ? 1u : 0u);
  }
  return result;
}

void head_dxl_abort_telemetry(struct head_runtime *runtime)
{
  uint32_t wait_ms = 0u;
  const uint32_t now_ms = k_uptime_get_32();
  uint8_t discarded[128];

  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    struct head_branch_health *health = &runtime->branches[branch_index];
    if (health->telemetry_active) {
      const uint32_t elapsed = now_ms - health->telemetry_requested_ms;
      if (elapsed < HEAD_TELEMETRY_RESPONSE_TIMEOUT_MS) {
        const uint32_t remaining = HEAD_TELEMETRY_RESPONSE_TIMEOUT_MS - elapsed;
        if (remaining > wait_ms) wait_ms = remaining;
      }
    }
  }
  if (wait_ms != 0u) k_sleep(K_MSEC(wait_ms));

  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    runtime->branches[branch_index].telemetry_active = 0u;
    runtime->branches[branch_index].telemetry_received_mask = 0u;
    telemetry_rx_used[branch_index] = 0u;
    (void)head_board_branch_set_tx(branch_index, false);
    while (head_board_branch_read_available(branch_index, discarded, sizeof(discarded)) != 0u) {
    }
  }
}

size_t head_dxl_build_sync_write(uint8_t branch_index, const struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t *output, size_t capacity)
{
  size_t byte_offset = 0u;
  uint8_t member_count = 0u;
  if (branch_index >= HEAD_BRANCH_COUNT || capacity < 64u) return 0u;
  output[byte_offset++] = 0xFFu; output[byte_offset++] = 0xFFu; output[byte_offset++] = 0xFDu; output[byte_offset++] = 0u;
  output[byte_offset++] = DXL_BROADCAST_ID; output[byte_offset++] = 0u; output[byte_offset++] = 0u;
  output[byte_offset++] = 0x83u;             /* Sync Write */
  output[byte_offset++] = 116u; output[byte_offset++] = 0u; /* Goal Position */
  output[byte_offset++] = 4u; output[byte_offset++] = 0u;
  for (uint8_t offset = 0; offset < HEAD_SERVOS_PER_BRANCH; ++offset) {
    const uint8_t index = branch_index * HEAD_SERVOS_PER_BRANCH + offset;
    if (!servo_active(calibration, index)) continue;
    const int32_t target = runtime->servos[index].goal_tick;
    output[byte_offset++] = calibration->joints[index].servo_id;
    const uint32_t target_bits = (uint32_t)target;
    for (uint8_t byte = 0u; byte < 4u; ++byte) {
      output[byte_offset++] = (uint8_t)(target_bits >> (8u * byte));
    }
    ++member_count;
  }
  if (member_count == 0u) return 0u;
  const uint16_t packet_length = 3u + 4u + (uint16_t)member_count * 5u;
  output[5] = (uint8_t)packet_length;
  output[6] = (uint8_t)(packet_length >> 8u);
  return dxl_finish(output, byte_offset, capacity);
}

int head_dxl_write_targets(struct head_runtime *runtime,
                           const struct head_calibration *calibration,
                           uint8_t *written_branch_mask)
{
  for (uint8_t servo_index = 0u; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if (!servo_active(calibration, servo_index)) continue;
    if (runtime->servos[servo_index].goal_tick < HEAD_DXL_POSITION_MIN_TICK ||
        runtime->servos[servo_index].goal_tick > HEAD_DXL_POSITION_MAX_TICK) return -ERANGE;
  }
  /* Goal Position is normally streamed every control cycle, which is correct
   * while Profile Velocity and Acceleration are zero: each write just means
   * "be here now". Zero homing instead gives the servo a profile and a single
   * destination, and every write of Goal Position restarts that trajectory --
   * so rewriting an unchanged goal five hundred times a second would leave the
   * servo perpetually re-entering its acceleration phase and never decelerating
   * into the target. Send it only when it actually changes. */
  if (runtime->zero_homing && transmitted_goal_valid) {
    bool changed = false;
    for (uint8_t servo_index = 0u; servo_index < HEAD_SERVO_COUNT && !changed; ++servo_index) {
      if (!servo_active(calibration, servo_index)) continue;
      changed = runtime->servos[servo_index].goal_tick != transmitted_goal_tick[servo_index];
    }
    if (!changed) {
      if (written_branch_mask != NULL) *written_branch_mask = 0u;
      return 0;
    }
  }
  uint8_t packet[HEAD_BRANCH_COUNT][64];
  bool started[HEAD_BRANCH_COUNT] = { false };
  int result = 0;
  uint8_t written = 0u;
  for (uint8_t branch_index = 0; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (runtime->branches[branch_index].telemetry_active) {
      ++runtime->branches[branch_index].control_transmission_deferred;
      continue;
    }
    const size_t size = head_dxl_build_sync_write(branch_index, runtime, calibration,
                                                   packet[branch_index], sizeof(packet[branch_index]));
    if (size == 0u) continue;
    if (head_board_branch_write_async(branch_index, packet[branch_index], size) != 0) {
      ++runtime->branches[branch_index].control_transmission_errors;
      result = -EIO;
    } else {
      started[branch_index] = true;
    }
  }
  for (uint8_t branch_index = 0; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (!started[branch_index]) continue;
    if (head_board_branch_write_wait(branch_index, 3000u) != 0) {
      ++runtime->branches[branch_index].control_transmission_errors;
      result = -EIO;
    } else {
      ++runtime->branches[branch_index].control_transmissions;
      written |= (uint8_t)(1u << branch_index);
    }
  }
  if (result == 0) {
    /* Only a clean sweep is recorded: a failed branch must be retried, not
     * suppressed as already sent. A branch deferred because its telemetry
     * Sync Read was still in flight is equally unsent, but it `continue`s
     * without touching `result`, so recording on `result == 0` alone marked
     * goals as transmitted that never reached the wire. Under zero homing the
     * suppression above then saw an unchanged goal and never rewrote it: the
     * servo held whatever preparation had written -- its own start position --
     * and homing ran out its timeout without the joint ever moving. Record
     * only the branches that actually transmitted, so a deferred one keeps its
     * previous transmitted goal, still compares as changed, and is retried on
     * the next cycle.
     *
     * The window scales with branch population: a one-servo Sync Read occupies
     * about 1.1 ms of each 10 ms telemetry period and a five-servo one about
     * 5.1 ms, so a single servo usually slipped through and a full branch
     * usually did not. */
    for (uint8_t servo_index = 0u; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
      const uint8_t branch_index = servo_index / HEAD_SERVOS_PER_BRANCH;
      if ((written & (uint8_t)(1u << branch_index)) == 0u) continue;
      transmitted_goal_tick[servo_index] = runtime->servos[servo_index].goal_tick;
    }
    transmitted_goal_valid = true;
    transmitted_goal_ms = k_uptime_get_32();
  }
  if (written_branch_mask != NULL) *written_branch_mask = written;
  return result;
}

static int dxl_read_status(uint8_t branch_index, uint8_t *response, size_t capacity,
                           uint32_t timeout_us, size_t *response_length)
{
  uint16_t packet_length;
  const size_t header_length = 7u;
  size_t total;

  if (capacity < 11u || response_length == NULL ||
      head_board_branch_read(branch_index, response, header_length, timeout_us) != 0) {
    return -EIO;
  }
  if (response[0] != 0xFFu || response[1] != 0xFFu || response[2] != 0xFDu ||
      response[3] != 0u) {
    return -EPROTO;
  }
  packet_length = (uint16_t)response[5] | ((uint16_t)response[6] << 8u);
  total = header_length + packet_length;
  if (packet_length < 4u || total > capacity ||
      head_board_branch_read(branch_index, &response[header_length], packet_length, timeout_us) != 0) {
    return -EIO;
  }
  *response_length = total;
  return 0;
}

static int dxl_read_register_once(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                                  uint16_t length, uint8_t *data)
{
  uint8_t request[20];
  uint8_t response[64];
  uint8_t parameters[48];
  uint8_t discarded[64];
  size_t response_length = 0u;
  const size_t request_length = dxl_read(servo_id, address, length, request, sizeof(request));
  if (request_length == 0u || data == NULL || length + 1u > sizeof(parameters)) return -EINVAL;
  while (head_board_branch_read_available(branch_index, discarded, sizeof(discarded)) != 0u) {
  }
  if (head_board_branch_write(branch_index, request, request_length) != 0 ||
      dxl_read_status(branch_index, response, sizeof(response), 3000u, &response_length) != 0 ||
      !dxl_status_valid(response, response_length, servo_id)) return -EIO;
  const size_t parameter_length = dxl_unstuff_status_parameters(
      response, response_length, parameters, sizeof(parameters));
  if (parameter_length != (size_t)length + 1u) return -EPROTO;
  if ((parameters[0] & 0x80u) != 0u && servo_id < HEAD_SERVO_COUNT) {
    hardware_alert_servo_mask |= 1u << servo_id;
  }
  if ((parameters[0] & 0x7fu) != 0u) {
    note_transaction_error(address, parameters[0]);
    return -EPROTO;
  }
  if ((parameters[0] & 0x80u) != 0u && !alert_tolerant_configuration &&
      !(address == 64u && length == 1u && parameters[1] == 0u)) {
    note_transaction_error(address, parameters[0]);
    return -EIO;
  }
  memcpy(data, &parameters[1], length);
  return 0;
}

static int dxl_write_register_u8(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                                 uint8_t value)
{
  uint8_t request[20];
  uint8_t response[32];
  uint8_t parameters[8];
  size_t response_length = 0u;
  const size_t request_length = dxl_write(servo_id, address, &value, sizeof(value),
                                           request, sizeof(request));
  if (request_length == 0u ||
      head_board_branch_write(branch_index, request, request_length) != 0 ||
      dxl_read_status(branch_index, response, sizeof(response), 3000u, &response_length) != 0 ||
      !dxl_status_valid(response, response_length, servo_id)) return -EIO;
  const size_t parameter_length = dxl_unstuff_status_parameters(
      response, response_length, parameters, sizeof(parameters));
  const uint8_t status = alert_tolerant_configuration ?
      (uint8_t)(parameters[0] & 0x7fu) : parameters[0];
  if (parameter_length != 1u || status != 0u) {
    note_transaction_error(address, parameter_length == 1u ? parameters[0] : 0xFFu);
    return -EPROTO;
  }
  return 0;
}

static int dxl_write_register_bytes(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                                    const uint8_t *data, uint16_t length)
{
  uint8_t request[32];
  uint8_t response[32];
  uint8_t parameters[8];
  size_t response_length = 0u;
  const size_t request_length = dxl_write(servo_id, address, data, length,
                                           request, sizeof(request));
  if (request_length == 0u ||
      head_board_branch_write(branch_index, request, request_length) != 0 ||
      dxl_read_status(branch_index, response, sizeof(response), 3000u, &response_length) != 0 ||
      !dxl_status_valid(response, response_length, servo_id)) return -EIO;
  const size_t parameter_length = dxl_unstuff_status_parameters(
      response, response_length, parameters, sizeof(parameters));
  const uint8_t status = alert_tolerant_configuration ?
      (uint8_t)(parameters[0] & 0x7fu) : parameters[0];
  if (parameter_length != 1u || status != 0u) {
    note_transaction_error(address, parameter_length == 1u ? parameters[0] : 0xFFu);
    return -EPROTO;
  }
  return 0;
}

/* Roughly one transfer in a hundred is lost on this bus without any protocol
 * error, and a single-shot sequence multiplies that: twenty servos of seven
 * transfers each would abort more often than it completes. Retry only where
 * there is no reply to interpret. A status error is the servo rejecting the
 * request on purpose, so repeating it would change nothing and hide a real
 * rejection. Enabled only for callers that are not on a control deadline;
 * preparation retries a whole step across control cycles instead. */
static int dxl_read_register(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                             uint16_t length, uint8_t *data)
{
  int result = -EIO;
  for (uint8_t attempt = 0u; attempt <= read_retry_attempts; ++attempt) {
    result = dxl_read_register_once(branch_index, servo_id, address, length, data);
    if (result != -EIO) break;
    if (attempt < read_retry_attempts && read_retry_count < UINT16_MAX) ++read_retry_count;
  }
  return result;
}

static int dxl_ensure_register(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                               const uint8_t *expected, uint16_t length)
{
  uint8_t actual[8];
  if (expected == NULL || length == 0u || length > sizeof(actual)) return -EINVAL;
  if (dxl_read_register(branch_index, servo_id, address, length, actual) != 0) return -EIO;
  if (memcmp(actual, expected, length) == 0) return 0;
  if (dxl_write_register_bytes(branch_index, servo_id, address, expected, length) != 0) return -EIO;
  /* An EEPROM write is acknowledged before the cell is committed, so reading
   * straight back can still return the previous value. That is indisputably a
   * success being reported as -EIO: the servo raised no protocol error, and
   * the same write lands on the next discovery pass. Only EEPROM pays the
   * wait; a RAM register that disagrees after its write is a real mismatch and
   * still fails on the first readback. Bounded well inside the 500 ms
   * per-servo budget that discover_inventory_locked() enforces. */
  for (uint8_t attempt = 0u; ; ++attempt) {
    if (dxl_read_register(branch_index, servo_id, address, length, actual) != 0) return -EIO;
    if (memcmp(actual, expected, length) == 0) return 0;
    if (address >= DXL_EEPROM_ADDRESS_END || attempt >= DXL_EEPROM_COMMIT_RETRIES) return -EIO;
    k_sleep(K_MSEC(DXL_EEPROM_COMMIT_DELAY_MS));
  }
}

/* Preparation never runs an inventory-sized readback loop in one control
 * iteration. The owner checks cancellation, lease and a separate deadline
 * between steps. All servos are verified off before goals are changed. */
static int prepare_step_once(struct head_runtime *runtime,
                          const struct head_calibration *calibration)
{
  if (runtime->preparation_phase == 0u) {
    /* Nothing on the wire yet for this run. */
    transmitted_goal_valid = false;
    const int result = head_dxl_emergency_torque_off();
    if (result != 0) return result;
    runtime->preparation_phase = 1u;
    runtime->preparation_servo_index = 0u;
    return 0;
  }
  const bool homing = runtime->preparation_target_state == HEAD_HOMING ||
                      runtime->preparation_target_state == HEAD_MAINTENANCE_CALIBRATION;
  uint8_t servo_index = runtime->preparation_servo_index;
  if (homing && runtime->preparation_phase > 2u) {
    servo_index = servo_index == 0u ? runtime->homing_index : HEAD_SERVO_COUNT;
  }
  while (servo_index < HEAD_SERVO_COUNT && !servo_active(calibration, servo_index)) ++servo_index;
  if (servo_index == HEAD_SERVO_COUNT) {
    runtime->preparation_servo_index = 0u;
    if (++runtime->preparation_phase == 8u) return 1;
    return 0;
  }
  runtime->preparation_servo_index = (uint8_t)(servo_index + 1u);
  const struct head_joint_config *joint = &calibration->joints[servo_index];
  if (homing && runtime->preparation_phase > 2u) runtime->preparation_servo_index = HEAD_SERVO_COUNT;
  uint8_t actual[8];
  uint8_t expected[8];
  uint16_t address = 0u;
  uint16_t length = 1u;
  switch (runtime->preparation_phase) {
  case 1:
    if (dxl_read_register(joint->branch_index, joint->servo_id, 64u, 1u, actual) != 0 || actual[0] != 0u) return -EIO;
    return 0;
  case 2: address = 98u; expected[0] = 0u; break;
  case 3:
    if (dxl_read_register(joint->branch_index, joint->servo_id, 144u, 2u, actual) != 0) return -EIO;
    const uint32_t voltage_mv = ((uint32_t)actual[0] | ((uint32_t)actual[1] << 8u)) * 100u;
    if (voltage_mv < DXL_MINIMUM_SAFE_VOLTAGE_MV || voltage_mv > DXL_MAXIMUM_SAFE_VOLTAGE_MV) return -ERANGE;
    return 0;
  case 4: {
    /* Profile Acceleration (108) and Profile Velocity (112) are contiguous and
     * both zero from the factory. A stop search needs them zero because it
     * shapes the motion itself, walking Goal Position one tick per control
     * cycle. Zero homing writes a single destination instead, which with the
     * profile at zero is exactly what DYNAMIXEL Wizard does. Re-asserted every
     * preparation so nothing carries over between runs. */
    address = 108u; length = 8u;
    /* Zero means unlimited on a DYNAMIXEL, not stopped: with both at zero the
     * servo drives at Velocity Limit (~117 rev/min, ~700 deg/s at the output)
     * and arrives at full speed. A stop search still wants that, because it
     * shapes the motion itself one tick at a time. Zero homing writes a single
     * destination, so give the servo a trajectory to follow instead of letting
     * it slam into the target. Redundant Goal Position writes are suppressed
     * for this path, or each one would restart the profile. */
    const uint32_t acceleration = runtime->zero_homing ?
        DXL_ZERO_HOME_PROFILE_ACCELERATION : 0u;
    const uint32_t velocity = runtime->zero_homing ?
        DXL_ZERO_HOME_PROFILE_VELOCITY : 0u;
    for (uint8_t byte = 0u; byte < 4u; ++byte) {
      expected[byte] = (uint8_t)(acceleration >> (8u * byte));
      expected[4u + byte] = (uint8_t)(velocity >> (8u * byte));
    }
    break;
  }
  case 5:
    address = 102u; length = 2u;
    /* The homing limit exists to make a deliberate collision survivable: a stop
     * search drives the joint into its endstop and reads the current rise as
     * the detection itself. Zero homing collides with nothing -- it drives to a
     * known encoder position -- so that limit protects against nothing here and
     * costs a great deal. Set low enough it can saturate the current loop just
     * overcoming the gearbox, which makes the servo stick, break free and
     * overshoot rather than track the ramp. */
    const bool collision_limited = homing && !runtime->zero_homing;
    const uint16_t current_ma = (uint16_t)(collision_limited ?
        joint->homing_current_limit_ma : joint->operating_current_ma);
    expected[0] = (uint8_t)current_ma; expected[1] = (uint8_t)(current_ma >> 8u);
    break;
  case 6:
    /* The servo's Bus Watchdog stops the motor when no goal update arrives in
     * its window, which suits a stop search: that streams Goal Position every
     * control cycle anyway, so the watchdog costs nothing and catches a dead
     * master. Zero homing writes its destination once and lets the servo's own
     * profile drive there, so there is nothing to feed the watchdog with --
     * and refreshing the goal just to feed it restarts the profile, which is
     * itself the oscillation. Leave it off for that path; the host lease still
     * bounds a dead master, and stale telemetry still faults. */
    address = 98u;
    expected[0] = runtime->zero_homing ? 0u : DXL_BUS_WATCHDOG_200_MS;
    break;
  case 7: {
    /* A mechanically coupled, unpowered joint may have moved since the last
     * stage. Read its position now, then verify that exact goal before torque. */
    if (dxl_read_register(joint->branch_index, joint->servo_id, 132u, 4u, actual) != 0) return -EIO;
    const int32_t present_tick = (int32_t)((uint32_t)actual[0] | ((uint32_t)actual[1] << 8u) |
        ((uint32_t)actual[2] << 16u) | ((uint32_t)actual[3] << 24u));
    if (present_tick < HEAD_DXL_POSITION_MIN_TICK || present_tick > HEAD_DXL_POSITION_MAX_TICK) return -ERANGE;
    if (dxl_write_register_bytes(joint->branch_index, joint->servo_id, 116u, actual, 4u) != 0) return -EIO;
    memcpy(expected, actual, 4u);
    if (dxl_read_register(joint->branch_index, joint->servo_id, 116u, 4u, actual) != 0 || memcmp(actual, expected, 4u) != 0) return -EIO;
    runtime->servos[servo_index].present_tick = present_tick;
    runtime->servos[servo_index].goal_tick = present_tick;
    runtime->servos[servo_index].goal_step_ticks_per_control_cycle = 0;
    runtime->servos[servo_index].last_feedback_ms = k_uptime_get_32();
    /* Mark uncertainty before the write; even a lost ACK can leave torque on. */
    runtime->torque_state = HEAD_TORQUE_SHUTDOWN_PENDING;
    address = 64u; expected[0] = 1u;
    break;
  }
  default: return -EINVAL;
  }
  if (dxl_write_register_bytes(joint->branch_index, joint->servo_id, address, expected, length) != 0 ||
      dxl_read_register(joint->branch_index, joint->servo_id, address, length, actual) != 0 ||
      memcmp(actual, expected, length) != 0) return -EIO;
  return 0;
}

/* One lost transfer must not abort a torque-on sequence outright. Retrying
 * inline is not available here: the owner aborts a step that exceeds 20 ms, so
 * repeated 3 ms reads would trip the control deadline instead. Preparation is
 * already incremental, so repeat the whole step on the next control cycle by
 * rewinding the servo cursor and reporting "not complete yet". Every step is
 * idempotent -- reads have no effect, and each write is re-verified -- so a
 * repeat is safe even when the previous attempt's write landed. The 1500 ms
 * preparation deadline bounds the retries whatever happens, and a genuinely
 * unreachable servo still faults, just a few cycles later.
 *
 * -ERANGE and -EINVAL are never retried: an out-of-range voltage or position
 * is the servo answering correctly with an answer that forbids motion. */
int head_dxl_prepare_step(struct head_runtime *runtime,
                          const struct head_calibration *calibration)
{
  if (runtime == NULL || calibration == NULL) return -EINVAL;
  const uint8_t cursor = runtime->preparation_servo_index;
  const int result = prepare_step_once(runtime, calibration);
  if (result >= 0) {
    runtime->preparation_attempts = 0u;
    return result;
  }
  if (result == -ERANGE || result == -EINVAL) return result;
  if (runtime->preparation_attempts >= HEAD_PREPARATION_STEP_RETRIES) return result;
  /* Recover the receivers before retrying. A starved RX never reports
   * RX_DISABLED, so it stays silent until something re-arms it -- and the
   * telemetry timeout that normally does so is suppressed for the whole of
   * preparation. Without this every retry would question the same dead
   * receiver and the budget would drain for certain, which is exactly the
   * burst of consecutive losses seen here. The branches are idle apart from
   * this transfer, so re-arming all of them costs nothing. */
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    (void)head_board_branch_rx_restart(branch_index);
  }
  ++runtime->preparation_attempts;
  if (read_retry_count < UINT16_MAX) ++read_retry_count;
  runtime->preparation_servo_index = cursor;
  return 0;
}

uint16_t head_dxl_read_retry_count(void)
{
  return read_retry_count;
}

/* Status Return Level 1 deliberately produces no Write response.  This helper
 * is used only to change it to level 2; discovery then reads the register back
 * before relying on acknowledged writes. */
static int dxl_write_register_u8_without_status(uint8_t branch_index, uint8_t servo_id,
                                                uint16_t address, uint8_t value)
{
  uint8_t request[20];
  uint8_t discarded[32];
  const size_t request_length = dxl_write(servo_id, address, &value, sizeof(value),
                                           request, sizeof(request));
  if (request_length == 0u ||
      head_board_branch_write(branch_index, request, request_length) != 0) return -EIO;
  /* The maximum documented Return Delay is 508 us.  Wait past it and discard
   * an unexpected ACK before issuing the readback request. */
  k_busy_wait(750u);
  while (head_board_branch_read_available(branch_index, discarded, sizeof(discarded)) != 0u) {
  }
  return 0;
}

static int dxl_verify_register_u8_all(const struct head_calibration *calibration,
                                      uint16_t address, uint8_t expected)
{
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    uint8_t actual = 0u;
    if (!servo_active(calibration, index)) continue;
    const struct head_joint_config *joint = &calibration->joints[index];
    if (dxl_read_register(joint->branch_index, joint->servo_id, address, 1u, &actual) != 0 ||
        actual != expected) return -EIO;
  }
  return 0;
}

int head_dxl_debug_ping(uint8_t branch_index, uint8_t servo_id,
                        struct head_dxl_debug_ping *diagnostic)
{
  if (branch_index >= HEAD_BRANCH_COUNT || diagnostic == NULL) return -EINVAL;
  memset(diagnostic, 0, sizeof(*diagnostic));
  diagnostic->first_byte_delay_us = UINT32_MAX;
  diagnostic->request_length = (uint8_t)dxl_ping(servo_id, diagnostic->request,
                                                   sizeof(diagnostic->request));
  if (diagnostic->request_length == 0u) return -EINVAL;

  uint8_t discarded[64];
  size_t stale = 0u;
  size_t dropped;
  while ((dropped = head_board_branch_read_available(branch_index, discarded,
                                                     sizeof(discarded))) != 0u) {
    stale += dropped;
  }
  diagnostic->stale_bytes = stale > UINT16_MAX ? UINT16_MAX : (uint16_t)stale;
  if (head_board_branch_write(branch_index, diagnostic->request,
                              diagnostic->request_length) != 0) return -EIO;

  const uint32_t started = k_cycle_get_32();
  const uint32_t deadline = started + k_us_to_cyc_ceil32(DXL_DEBUG_PING_WINDOW_US);
  while ((int32_t)(k_cycle_get_32() - deadline) < 0) {
    const size_t room = sizeof(diagnostic->received) - diagnostic->received_length;
    if (room != 0u) {
      const size_t count = head_board_branch_read_available(
          branch_index, &diagnostic->received[diagnostic->received_length], room);
      if (count != 0u) {
        if (diagnostic->received_length == 0u)
          diagnostic->first_byte_delay_us = k_cyc_to_us_floor32(k_cycle_get_32() - started);
        diagnostic->received_length += (uint8_t)count;
      }
    }
    diagnostic->uart_error_flags |= head_board_branch_error_flags(branch_index);
    if (diagnostic->received_length == sizeof(diagnostic->received)) break;
    /* A short reply reaches the ring only when the UART driver's idle-timeout
     * work item runs on the cooperative system workqueue. Polling without
     * sleeping starves it: once the control thread blocks on the head lock,
     * priority inheritance raises this thread into the cooperative range and
     * the equal-priority workqueue can never preempt it. */
    const int32_t remaining_cycles = (int32_t)(deadline - k_cycle_get_32());
    if (remaining_cycles <= 0) break;
    (void)head_board_branch_wait_rx(branch_index,
        k_cyc_to_us_ceil32((uint32_t)remaining_cycles));
  }
  head_board_branch_rx_stats(branch_index, &diagnostic->rx_disabled_count,
                             &diagnostic->rx_restart_failures,
                             &diagnostic->rx_overflow_count);
  if (diagnostic->received_length == 0u) return 0;
  diagnostic->flags |= HEAD_DXL_DEBUG_RX_ANY;
  const uint8_t *packet = diagnostic->received;
  const size_t length = diagnostic->received_length;
  if (length >= 4u && packet[0] == 0xFFu && packet[1] == 0xFFu &&
      packet[2] == 0xFDu && packet[3] == 0u) diagnostic->flags |= HEAD_DXL_DEBUG_HEADER_OK;
  if (length >= 7u) {
    const size_t expected = 7u + (size_t)packet[5] + ((size_t)packet[6] << 8u);
    if (expected == length && expected >= 11u) diagnostic->flags |= HEAD_DXL_DEBUG_LENGTH_OK;
  }
  if (length >= 8u && packet[7] == 0x55u) diagnostic->flags |= HEAD_DXL_DEBUG_STATUS_OK;
  if (length >= 5u && packet[4] == servo_id) diagnostic->flags |= HEAD_DXL_DEBUG_ID_OK;
  if ((diagnostic->flags & HEAD_DXL_DEBUG_LENGTH_OK) != 0u &&
      dxl_status_valid(packet, length, servo_id)) diagnostic->flags |= HEAD_DXL_DEBUG_CRC_OK;
  return 0;
}

int head_dxl_debug_read(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                        uint8_t length, uint8_t *data)
{
  if (branch_index >= HEAD_BRANCH_COUNT || data == NULL || length == 0u ||
      length > 32u || servo_id >= DXL_BROADCAST_ID) return -EINVAL;
  /* An alerting servo refuses ordinary reads, and its Hardware Error Status is
   * exactly what an operator needs to see in that state. */
  const bool previous = alert_tolerant_configuration;
  alert_tolerant_configuration = true;
  const int result = dxl_read_register(branch_index, servo_id, address, length, data);
  alert_tolerant_configuration = previous;
  return result;
}

int head_dxl_probe_branch(uint8_t branch_index, struct head_dxl_probe_reply *replies,
                          size_t capacity, size_t *reply_count)
{
  uint8_t request[16];
  uint8_t response[32];
  uint8_t discarded[32];

  if (branch_index >= HEAD_BRANCH_COUNT || replies == NULL || reply_count == NULL) {
    return -EINVAL;
  }
  *reply_count = 0u;

  /* Unicast Pings are intentionally sequential. They avoid simultaneous
   * status packets if a branch accidentally has more than one servo. */
  for (uint16_t raw_id = 0u; raw_id < DXL_BROADCAST_ID; ++raw_id) {
    const uint8_t servo_id = (uint8_t)raw_id;
    size_t response_length = 0u;
    const size_t size = dxl_ping(servo_id, request, sizeof(request));

    (void)head_board_branch_read_available(branch_index, discarded, sizeof(discarded));
    if (size == 0u || head_board_branch_write(branch_index, request, size) != 0) {
      return -EIO;
    }
    if (dxl_read_status(branch_index, response, sizeof(response), 2500u,
                        &response_length) != 0 ||
        response_length < 14u ||
        !dxl_status_valid(response, response_length, servo_id) ||
        response[8] != 0u) {
      continue; /* No reply is normal for an unassigned ID. */
    }
    if (*reply_count >= capacity) return -ENOSPC;
    replies[*reply_count].servo_id = servo_id;
    replies[*reply_count].model_number =
        (uint16_t)response[9] | ((uint16_t)response[10] << 8u);
    replies[*reply_count].firmware_version = response[11];
    ++*reply_count;
  }
  return 0;
}

int head_dxl_discover(struct head_runtime *runtime,
                      const struct head_calibration *calibration)
{
  uint8_t request[16];
  uint8_t response[32];
  uint8_t online = 0u;

  alert_tolerant_configuration = true;
  read_retry_attempts = DXL_DISCOVERY_READ_RETRIES;
  for (uint8_t index = 0; index < HEAD_SERVO_COUNT; ++index) {
    const struct head_joint_config *joint = &calibration->joints[index];
    size_t response_length = 0u;
    if (!servo_active(calibration, index)) {
      discovery_reason[index] = HEAD_DXL_DISCOVERY_INACTIVE;
      continue;
    }
    const size_t size = dxl_ping(joint->servo_id, request, sizeof(request));
    runtime->servos[index].online = false;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_NO_REPLY;
    /* The ping is the first transaction against each servo and the only one
     * that does not go through dxl_read_register(), so it needs its own retry:
     * without it a single lost transfer still rejects a healthy servo as
     * NO_REPLY and fails the whole inventory. */
    bool ping_answered = false;
    for (uint8_t attempt = 0u; attempt <= DXL_DISCOVERY_READ_RETRIES && !ping_answered;
         ++attempt) {
      response_length = 0u;
      /* Discard anything already buffered, as every other transaction here
       * does. This is the first bus traffic after init and the broadcast
       * torque-off, and a single leftover byte shifts the status header by
       * one, which dxl_read_status rejects outright rather than
       * resynchronizing. */
      while (head_board_branch_read_available(joint->branch_index, response,
                                              sizeof(response)) != 0u) {
      }
      if (size != 0u &&
          head_board_branch_write(joint->branch_index, request, size) == 0 &&
          dxl_read_status(joint->branch_index, response, sizeof(response), 2500u,
                          &response_length) == 0 &&
          response_length >= 14u &&
          dxl_status_valid(response, response_length, joint->servo_id)) {
        ping_answered = true;
      } else if (attempt < DXL_DISCOVERY_READ_RETRIES && read_retry_count < UINT16_MAX) {
        ++read_retry_count;
      }
    }
    if (!ping_answered) continue;
    /* Bit 7 is the Protocol 2.0 Hardware Alert: the servo answered, but its
     * Hardware Error Status (address 70) is latched non-zero. Do not reject
     * here. An alert caused by a voltage limit this firmware itself wrote can
     * only be cleared by rewriting that limit, which happens below; rejecting
     * first would make the condition permanent. The alert is re-checked after
     * normalization, and a servo whose latch survives is still refused. */
    const bool hardware_alert = response[8] != 0u;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_MODEL;
    if (((uint16_t)response[9] | ((uint16_t)response[10] << 8u)) !=
            DXL_XC330_T181_MODEL_NUMBER) continue;

    const uint8_t firmware_version = response[11];
    uint8_t communication[6];
    uint8_t current_limit_bytes[2];
    uint8_t status_return_level;
    uint8_t torque_enable;
    uint8_t shutdown;
    uint8_t profiles[8];
    discovery_reason[index] = HEAD_DXL_DISCOVERY_FIRMWARE;
    if (firmware_version < DXL_MINIMUM_WATCHDOG_FIRMWARE) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_COMMUNICATION_READ;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 8u, sizeof(communication),
                          communication) != 0) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_BAUD;
    if (communication[0] != DXL_BAUD_1M) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_DRIVE_MODE;
    if ((communication[2] & 0x0Cu) != 0u) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_PROTOCOL;
    if (communication[5] != DXL_PROTOCOL_VERSION) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_TORQUE_ON;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 64u, 1u, &torque_enable) != 0 ||
        torque_enable != 0u) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_STATUS_LEVEL;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 68u, 1u,
                          &status_return_level) != 0) continue;
    if (status_return_level != DXL_STATUS_RETURN_ALL) {
      if (dxl_write_register_u8_without_status(joint->branch_index, joint->servo_id, 68u,
                                               DXL_STATUS_RETURN_ALL) != 0 ||
          dxl_read_register(joint->branch_index, joint->servo_id, 68u, 1u,
                            &status_return_level) != 0 ||
          status_return_level != DXL_STATUS_RETURN_ALL) continue;
    }
    discovery_reason[index] = HEAD_DXL_DISCOVERY_SECONDARY_ID;
    if (dxl_ensure_register(joint->branch_index, joint->servo_id, 12u,
                            (const uint8_t[]){ DXL_SECONDARY_ID_DISABLED }, 1u) != 0) {
      continue;
    }
    /* These are confirmed project-wide settings. Only write EEPROM when a
     * mismatch exists; the bus is torque-off throughout discovery. */
    discovery_reason[index] = HEAD_DXL_DISCOVERY_STARTUP;
    if (firmware_version >= DXL_STARTUP_CONFIGURATION_FIRMWARE) {
      uint8_t startup_configuration;
      if (dxl_read_register(joint->branch_index, joint->servo_id, 60u, 1u,
                            &startup_configuration) != 0) continue;
      if ((startup_configuration & DXL_STARTUP_TORQUE_ON_MASK) != 0u) {
        const uint8_t safe_startup = startup_configuration &
                                     (uint8_t)~DXL_STARTUP_TORQUE_ON_MASK;
        if (dxl_write_register_u8(joint->branch_index, joint->servo_id, 60u,
                                  safe_startup) != 0 ||
            dxl_read_register(joint->branch_index, joint->servo_id, 60u, 1u,
                              &startup_configuration) != 0 ||
            startup_configuration != safe_startup) continue;
      }
    }
    discovery_reason[index] = HEAD_DXL_DISCOVERY_RETURN_DELAY;
    if (communication[1] != 0u &&
        dxl_write_register_u8(joint->branch_index, joint->servo_id, 9u, 0u) != 0) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_OPERATING_MODE;
    if (communication[3] != DXL_OPERATING_MODE_CURRENT_POSITION &&
        dxl_write_register_u8(joint->branch_index, joint->servo_id, 11u,
                              DXL_OPERATING_MODE_CURRENT_POSITION) != 0) continue;
    uint8_t verified_return_delay;
    uint8_t verified_operating_mode;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_VERIFY_READBACK;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 9u, 1u,
                          &verified_return_delay) != 0 ||
        dxl_read_register(joint->branch_index, joint->servo_id, 11u, 1u,
                          &verified_operating_mode) != 0 ||
        verified_return_delay != 0u ||
        verified_operating_mode != DXL_OPERATING_MODE_CURRENT_POSITION) continue;
    const uint8_t current_limit_expected[2] = {
      (uint8_t)joint->operating_current_ma,
      (uint8_t)((uint16_t)joint->operating_current_ma >> 8u),
    };
    discovery_reason[index] = HEAD_DXL_DISCOVERY_LIMITS;
    if (dxl_ensure_register(joint->branch_index, joint->servo_id, 20u,
                            (const uint8_t[]){ 0u, 0u, 0u, 0u }, 4u) != 0 ||
        dxl_ensure_register(joint->branch_index, joint->servo_id, 24u,
                            (const uint8_t[]){ 10u, 0u, 0u, 0u }, 4u) != 0 ||
        dxl_ensure_register(joint->branch_index, joint->servo_id, 31u,
                            (const uint8_t[]){ 70u }, 1u) != 0 ||
        /* Max Voltage Limit is the servo's own instantaneous trip, not an
         * operating rating, so it is held at the ROBOTIS factory default of
         * 14.0 V rather than the 12.0 V steady-state maximum. A decelerating
         * motor regenerates into a rail a CV bench supply cannot sink, and on
         * an 11.1 V supply that kick clears 12.0 V easily -- latching an Input
         * Voltage Error (Hardware Error Status bit 0) on an entirely healthy
         * bench. Supply voltage is still held to the datasheet range: phase 3
         * of preparation range-checks Present Input Voltage against
         * DXL_MAXIMUM_SAFE_VOLTAGE_MV before any torque-on. */
        dxl_ensure_register(joint->branch_index, joint->servo_id, 32u,
                            (const uint8_t[]){ 140u, 0u }, 2u) != 0 ||
        dxl_ensure_register(joint->branch_index, joint->servo_id, 34u,
                            (const uint8_t[]){ 65u, 0u }, 2u) != 0 ||
        dxl_ensure_register(joint->branch_index, joint->servo_id, 36u,
                            (const uint8_t[]){ 0x75u, 0x03u }, 2u) != 0 ||
        dxl_ensure_register(joint->branch_index, joint->servo_id, 38u,
                            current_limit_expected, 2u) != 0 ||
        dxl_ensure_register(joint->branch_index, joint->servo_id, 63u,
                            (const uint8_t[]){ DXL_SHUTDOWN_REQUIRED_MASK }, 1u) != 0 ||
        dxl_read_register(joint->branch_index, joint->servo_id, 63u, 1u, &shutdown) != 0 ||
        shutdown != DXL_SHUTDOWN_REQUIRED_MASK) continue;
    const struct {
      uint16_t address;
      uint16_t value;
    } gains[] = {
      /* Position D (80) departs from the factory 0. A bare horn behind a
       * 180.62:1 gearbox reflects almost no inertia and has real lash, so a
       * P-only loop chases the output through the backlash band and sustains
       * its own oscillation -- damping it by hand collapses the cycle and it
       * then holds rigidly, which is the signature. D supplies that damping
       * instead of a finger. Harmless once a joint is loaded; necessary while
       * it is not. */
      { 76u, 1200u }, { 78u, 40u }, { 80u, 1000u }, { 82u, 0u },
      { 84u, 900u }, { 88u, 0u }, { 90u, 0u },
    };
    bool gains_valid = true;
    for (size_t gain = 0u; gain < sizeof(gains) / sizeof(gains[0]); ++gain) {
      const uint8_t expected_gain[2] = {
        (uint8_t)gains[gain].value, (uint8_t)(gains[gain].value >> 8u),
      };
      if (dxl_ensure_register(joint->branch_index, joint->servo_id, gains[gain].address,
                              expected_gain, sizeof(expected_gain)) != 0) {
        gains_valid = false;
        break;
      }
    }
    discovery_reason[index] = HEAD_DXL_DISCOVERY_GAINS;
    if (!gains_valid) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_PROFILES;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 108u, sizeof(profiles),
                          profiles) != 0) continue;
    const uint8_t zero_profiles[8] = { 0u };
    if (memcmp(profiles, zero_profiles, sizeof(profiles)) != 0 &&
        (dxl_write_register_bytes(joint->branch_index, joint->servo_id, 108u,
                                  zero_profiles, sizeof(zero_profiles)) != 0 ||
         dxl_read_register(joint->branch_index, joint->servo_id, 108u, sizeof(profiles),
                           profiles) != 0 ||
         memcmp(profiles, zero_profiles, sizeof(profiles)) != 0)) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_WATCHDOG;
    uint8_t bus_watchdog;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 98u, 1u,
                          &bus_watchdog) != 0) continue;
    if (bus_watchdog != 0u &&
        (dxl_write_register_u8(joint->branch_index, joint->servo_id, 98u, 0u) != 0 ||
         dxl_read_register(joint->branch_index, joint->servo_id, 98u, 1u,
                           &bus_watchdog) != 0 ||
         bus_watchdog != 0u)) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_CURRENT_LIMIT;
    if (dxl_read_register(joint->branch_index, joint->servo_id, 38u,
                          sizeof(current_limit_bytes), current_limit_bytes) != 0) continue;
    const uint16_t current_limit = (uint16_t)current_limit_bytes[0] |
                                   ((uint16_t)current_limit_bytes[1] << 8u);
    if (current_limit != (uint16_t)joint->operating_current_ma) continue;
    /* Limits are now correct. A still-latched alert needs a servo power cycle
     * (Hardware Error Status only clears on reboot) before it can pass. */
    discovery_reason[index] = HEAD_DXL_DISCOVERY_STATUS_ERROR;
    if (hardware_alert) continue;
    discovery_reason[index] = HEAD_DXL_DISCOVERY_OK;
    runtime->servos[index].firmware_version = firmware_version;
    runtime->servos[index].online = true;
    ++online;
  }
  alert_tolerant_configuration = false;
  read_retry_attempts = 0u;
  return online == calibration->expected_servo_count ? 0 : -ENODEV;
}

/* The caller serializes bus ownership. A static subset avoids placing a
 * full calibration object on the small control/worker stack. */
int head_dxl_discover_one(struct head_runtime *runtime,
                          const struct head_calibration *calibration,
                          uint8_t servo_index)
{
  static struct head_calibration discovery_subset;
  if (servo_index >= HEAD_SERVO_COUNT || !servo_active(calibration, servo_index)) return -EINVAL;
  discovery_subset = *calibration;
  discovery_subset.active_servo_mask = 1u << servo_index;
  discovery_subset.expected_servo_count = 1u;
  return head_dxl_discover(runtime, &discovery_subset);
}

static void telemetry_consume(struct head_runtime *runtime,
                              const struct head_calibration *calibration,
                              uint8_t branch_index, uint32_t now_ms)
{
  uint8_t incoming[128];
  size_t *used = &telemetry_rx_used[branch_index];
  struct head_branch_health *health = &runtime->branches[branch_index];
  size_t count;
  do {
    count = head_board_branch_read_available(branch_index, incoming, sizeof(incoming));
    if (count > sizeof(telemetry_rx[branch_index]) - *used) {
      *used = 0u;
      ++health->protocol_errors;
    }
    memcpy(&telemetry_rx[branch_index][*used], incoming, count);
    *used += count;
  } while (count == sizeof(incoming));
  while (*used >= 7u) {
    uint16_t length;
    size_t total;
    if (telemetry_rx[branch_index][0] != 0xFFu || telemetry_rx[branch_index][1] != 0xFFu ||
        telemetry_rx[branch_index][2] != 0xFDu || telemetry_rx[branch_index][3] != 0u) {
      memmove(telemetry_rx[branch_index], &telemetry_rx[branch_index][1], --*used);
      continue;
    }
    length = (uint16_t)telemetry_rx[branch_index][5] | ((uint16_t)telemetry_rx[branch_index][6] << 8u);
    total = 7u + length;
    if (length < 4u || total > sizeof(telemetry_rx[branch_index])) {
      memmove(telemetry_rx[branch_index], &telemetry_rx[branch_index][1], --*used);
      ++health->protocol_errors;
      continue;
    }
    if (*used < total) break;
    const uint8_t servo_id = telemetry_rx[branch_index][4];
    const int servo_index = servo_index_for_id(calibration, branch_index, servo_id);
    uint8_t parameters[96];
    const size_t parameter_length = dxl_unstuff_status_parameters(telemetry_rx[branch_index],
                                                                     total, parameters,
                                                                     sizeof(parameters));
    if (servo_index >= 0 && dxl_status_valid(telemetry_rx[branch_index], total, servo_id) &&
        parameter_length == 78u && (parameters[0] & 0x7fu) == 0u) {
      const uint8_t index = (uint8_t)servo_index;
      struct head_servo_state *state = &runtime->servos[index];
      state->online = true;
      /* parameters[0] is the Protocol 2 Status Packet Error byte. It is
       * transport/instruction feedback, not Hardware Error Status(70). */
      if ((parameters[0] & 0x80u) != 0u) head_state_fault(runtime, HEAD_FAULT_SERVO);
      state->hardware_error = parameters[1];              /* address 70 */
      state->moving_status = parameters[54];              /* address 123 */
      state->present_current_ma = (int16_t)((uint16_t)parameters[57] |
          ((uint16_t)parameters[58] << 8u));
      state->present_velocity_raw = (int32_t)((uint32_t)parameters[59] |
          ((uint32_t)parameters[60] << 8u) |
          ((uint32_t)parameters[61] << 16u) |
          ((uint32_t)parameters[62] << 24u));
      state->present_tick = (int32_t)((uint32_t)parameters[63] |
          ((uint32_t)parameters[64] << 8u) |
          ((uint32_t)parameters[65] << 16u) |
          ((uint32_t)parameters[66] << 24u));
      state->present_voltage_mv = (uint16_t)parameters[75] |
          ((uint16_t)parameters[76] << 8u);
      state->present_voltage_mv *= 100u; /* register unit is 0.1 V */
      state->temperature_c = parameters[77];              /* address 146 */
      state->last_feedback_ms = now_ms;
      state->last_current_feedback_ms = now_ms;
      health->telemetry_received_mask |= (uint8_t)(1u << (index % HEAD_SERVOS_PER_BRANCH));
    } else {
      ++health->protocol_errors;
    }
    memmove(telemetry_rx[branch_index], &telemetry_rx[branch_index][total], *used - total);
    *used -= total;
  }
}

void head_dxl_telemetry_tick(struct head_runtime *runtime,
                             const struct head_calibration *calibration,
                             uint32_t now_ms, bool start_new)
{
  uint8_t request[32];
  for (uint8_t branch_index = 0; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    struct head_branch_health *health = &runtime->branches[branch_index];
    if (head_board_branch_error_flags(branch_index) != 0u) ++health->bus_errors;
    if (health->telemetry_active) {
      telemetry_consume(runtime, calibration, branch_index, now_ms);
      if (health->telemetry_received_mask == health->telemetry_expected_mask) {
        health->telemetry_active = false;
        health->telemetry_completed_ms = now_ms;
      } else if (now_ms - health->telemetry_requested_ms >= HEAD_TELEMETRY_RESPONSE_TIMEOUT_MS) {
        const uint8_t missing = health->telemetry_expected_mask &
                                (uint8_t)~health->telemetry_received_mask;
        for (uint8_t offset = 0u; offset < HEAD_SERVOS_PER_BRANCH; ++offset) {
          if ((missing & (1u << offset)) != 0u) {
            runtime->servos[branch_index * HEAD_SERVOS_PER_BRANCH + offset].online = false;
          }
        }
        ++health->telemetry_timeouts;
        health->telemetry_active = false;
        /* A branch whose receiver has starved never reports RX_DISABLED, so
         * head_board_branch_read_available() never re-arms it and every later
         * poll times out identically until reboot. Recover it here: the reply
         * for this cycle is already lost, and a healthy branch that simply
         * missed one reply is re-armed harmlessly. */
        if (head_board_branch_rx_restart(branch_index) != 0) ++health->bus_errors;
      }
      continue;
    }
    if (!start_new) continue;
    if (now_ms - health->telemetry_requested_ms < HEAD_TELEMETRY_PERIOD_MS) continue;
    const size_t size = dxl_sync_read(branch_index, calibration, request, sizeof(request));
    if (size == 0u) continue;
    health->telemetry_expected_mask = (uint8_t)((calibration->active_servo_mask >>
                                      (branch_index * HEAD_SERVOS_PER_BRANCH)) & 0x1Fu);
    health->telemetry_received_mask = 0u;
    telemetry_rx_used[branch_index] = 0u;
    uint8_t discarded[128];
    while (head_board_branch_read_available(branch_index, discarded, sizeof(discarded)) != 0u) {
    }
    health->telemetry_requested_ms = now_ms;
    if (head_board_branch_write(branch_index, request, size) != 0) {
      ++health->bus_errors;
      continue;
    }
    health->telemetry_active = true;
  }
}
