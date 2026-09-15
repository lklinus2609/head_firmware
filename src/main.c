#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/random/random.h>

#include "board.h"
#include "config.h"
#include "control.h"
#include "dxl.h"
#include "fan.h"
#include "lease.h"
#include "protocol.h"
#include "state_machine.h"

static struct head_calibration calibration;
static struct head_calibration staged_calibration;
static struct head_runtime runtime;
static const struct device *usb;
static struct head_frame_parser parser;
K_MUTEX_DEFINE(head_lock);
static uint32_t staged_slots_mask;
static bool staging_active;
static struct head_calibration storage_pending_calibration;
static bool storage_pending_apply;
#if !defined(HEAD_BENCH_NO_12V)
static bool worker_threads_started;
#endif
static atomic_t usb_configured;
/* Set only after persistent configuration and hardware discovery have
 * completed. The USB control channel remains available throughout bring-up. */
static atomic_t boot_complete;
#if defined(CONFIG_WATCHDOG)
static const struct device *hardware_watchdog;
static int hardware_watchdog_channel = -1;
#endif
static uint32_t boot_reset_cause;
static uint32_t boot_session_id;
static uint32_t lease_nonce;

struct diagnostic_request {
  uint8_t message_type;
  uint8_t branch_index;
  uint8_t transmit_enabled;
  uint32_t transaction_id;
};
static struct diagnostic_request pending_diagnostic;
K_SEM_DEFINE(diagnostic_pending_sem, 0, 1);

#define HEAD_USB_TX_QUEUE_DEPTH 8u
struct usb_tx_slot {
  uint8_t data[HEAD_MAX_FRAME_PAYLOAD + 9u];
  uint16_t length;
  uint8_t type;
  bool used;
  bool priority;
  uint32_t order;
};
static struct usb_tx_slot usb_tx_queue[HEAD_USB_TX_QUEUE_DEPTH];
static uint32_t usb_tx_order;
static atomic_t usb_tx_dropped_frames;
static atomic_t usb_tx_dropped_bytes;
static atomic_t usb_tx_coalesced_frames;
K_MUTEX_DEFINE(usb_tx_lock);
K_SEM_DEFINE(usb_tx_pending, 0, HEAD_USB_TX_QUEUE_DEPTH);
K_SEM_DEFINE(storage_pending_sem, 0, 1);
K_SEM_DEFINE(discovery_pending_sem, 0, 1);
static uint32_t discovery_transaction_id;

/* Fatal exceptions run with interrupts locked, so normal bus transactions are
 * unsafe here. Releasing every half-duplex driver lets the already-armed
 * actuator watchdogs stop motion; they do NOT guarantee torque removal.
 * Independent torque removal requires an external power disconnect. */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
  ARG_UNUSED(reason);
  ARG_UNUSED(esf);
  (void)head_board_release_all();
#if defined(CONFIG_REBOOT)
  sys_reboot(SYS_REBOOT_COLD);
  CODE_UNREACHABLE;
#else
  for (;;) { }
#endif
}

/* A build without the watchdog subsystem is a deliberate bench configuration,
 * not a runtime failure, so it must not fault at boot. Enforce the guarantee
 * where it belongs instead: any image that is not one of the bench profiles
 * fails to compile unless CONFIG_WATCHDOG is enabled. That is strictly
 * stronger than the boot-time fault it replaces -- a production image can no
 * longer be built without a watchdog at all, rather than being built and then
 * reporting it. */
#if !defined(CONFIG_WATCHDOG) && !defined(HEAD_BENCH_NO_12V) && \
    !defined(HEAD_BENCH_NO_FAN) && !defined(HEAD_BENCH_J3_ID10)
#error "Production images require CONFIG_WATCHDOG; add prj_production_storage.conf"
#endif

#if !defined(HEAD_BENCH_NO_12V) && defined(CONFIG_WATCHDOG)
static int hardware_watchdog_start(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(wdog0), okay)
  const struct wdt_timeout_cfg timeout = {
    .window = { .min = 0u, .max = 1000u },
    .callback = NULL,
    .flags = WDT_FLAG_RESET_SOC,
  };
  hardware_watchdog = DEVICE_DT_GET(DT_NODELABEL(wdog0));
  if (!device_is_ready(hardware_watchdog)) return -ENODEV;
  hardware_watchdog_channel = wdt_install_timeout(hardware_watchdog, &timeout);
  if (hardware_watchdog_channel < 0) return hardware_watchdog_channel;
  return wdt_setup(hardware_watchdog, WDT_OPT_PAUSE_HALTED_BY_DBG);
#else
  return -ENOTSUP;
#endif
}
#endif

static uint16_t get_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
}

static uint32_t get_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
         ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
}

static float get_f32_le(const uint8_t *data)
{
  const uint32_t bits = get_u32_le(data);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

/* A CDC ACM endpoint exists before a host application opens the serial port.
 * Do not queue telemetry during USB enumeration: on the legacy Zephyr USB
 * stack that can fill the endpoint before the descriptor exchange completes.
 * The host asserts DTR when head_ros/headctl opens the port. */
static bool usb_host_ready(void)
{
  uint32_t dtr = 0u;
  return usb != NULL && atomic_get(&usb_configured) != 0 &&
         uart_line_ctrl_get(usb, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr != 0u;
}

static void usb_status_callback(enum usb_dc_status_code status, const uint8_t *param)
{
  ARG_UNUSED(param);
  if (status == USB_DC_CONFIGURED) {
    atomic_set(&usb_configured, 1);
  } else if (status == USB_DC_DISCONNECTED || status == USB_DC_RESET) {
    atomic_clear(&usb_configured);
  }
}

static bool token_matches(const struct head_frame *frame, uint32_t now_ms)
{
  if (frame->length < sizeof(uint32_t)) return false;
  return head_lease_token_is_valid(&runtime, get_u32_le(frame->payload), now_ms);
}

static bool usb_low_priority_type(uint8_t type)
{
  return type == HEAD_MSG_STATE || type == HEAD_MSG_DIAGNOSTICS;
}

static bool usb_send(uint8_t type, const uint8_t *payload, uint16_t length)
{
  uint8_t encoded[HEAD_MAX_FRAME_PAYLOAD + 9u];
  int encoded_length = head_protocol_encode_frame(type, payload, length, encoded, sizeof(encoded));
  const bool priority = !usb_low_priority_type(type);
  int selected = -1;

  if (encoded_length < 0 || !usb_host_ready()) return false;
  k_mutex_lock(&usb_tx_lock, K_FOREVER);
  if (!priority) {
    for (uint8_t index = 0u; index < HEAD_USB_TX_QUEUE_DEPTH; ++index) {
      if (usb_tx_queue[index].used && !usb_tx_queue[index].priority &&
          usb_tx_queue[index].type == type) {
        memcpy(usb_tx_queue[index].data, encoded, (size_t)encoded_length);
        usb_tx_queue[index].length = (uint16_t)encoded_length;
        usb_tx_queue[index].order = ++usb_tx_order;
        atomic_inc(&usb_tx_coalesced_frames);
        k_mutex_unlock(&usb_tx_lock);
        return true;
      }
    }
  }
  for (uint8_t index = 0u; index < HEAD_USB_TX_QUEUE_DEPTH; ++index) {
    if (!usb_tx_queue[index].used) {
      selected = index;
      break;
    }
  }
  if (selected < 0 && priority) {
    for (uint8_t index = 0u; index < HEAD_USB_TX_QUEUE_DEPTH; ++index) {
      if (!usb_tx_queue[index].priority &&
          (selected < 0 || usb_tx_queue[index].order < usb_tx_queue[selected].order)) {
        selected = index;
      }
    }
    if (selected >= 0) {
      atomic_inc(&usb_tx_dropped_frames);
      atomic_add(&usb_tx_dropped_bytes, usb_tx_queue[selected].length);
    }
  }
  if (selected < 0) {
    atomic_inc(&usb_tx_dropped_frames);
    atomic_add(&usb_tx_dropped_bytes, encoded_length);
    k_mutex_unlock(&usb_tx_lock);
    return false;
  }
  struct usb_tx_slot *slot = &usb_tx_queue[selected];
  const bool was_used = slot->used;
  memcpy(slot->data, encoded, (size_t)encoded_length);
  slot->length = (uint16_t)encoded_length;
  slot->type = type;
  slot->priority = priority;
  slot->order = ++usb_tx_order;
  slot->used = true;
  k_mutex_unlock(&usb_tx_lock);
  if (!was_used) k_sem_give(&usb_tx_pending);
  return true;
}

static struct {
  struct k_spinlock lock;
  uint8_t frame[HEAD_MAX_FRAME_PAYLOAD + 9u];
  uint16_t length;
  uint16_t offset;
  int result;
  bool active;
} usb_output;
K_SEM_DEFINE(usb_output_done, 0, 1);

static void usb_uart_callback(const struct device *device, void *user_data)
{
  ARG_UNUSED(user_data);
  if (!uart_irq_update(device) || !uart_irq_tx_ready(device)) return;
  const k_spinlock_key_t key = k_spin_lock(&usb_output.lock);
  if (usb_output.active) {
    const int written = uart_fifo_fill(device, &usb_output.frame[usb_output.offset],
                                       usb_output.length - usb_output.offset);
    if (written > 0) usb_output.offset += (uint16_t)written;
    if (written < 0 || usb_output.offset == usb_output.length) {
      usb_output.result = written < 0 ? written : 0;
      usb_output.active = false;
      uart_irq_tx_disable(device);
      k_sem_give(&usb_output_done);
    }
  } else {
    uart_irq_tx_disable(device);
  }
  k_spin_unlock(&usb_output.lock, key);
}

static void usb_tx_thread(void *a, void *b, void *c)
{
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  uint8_t frame[HEAD_MAX_FRAME_PAYLOAD + 9u];

  while (true) {
    k_sem_take(&usb_tx_pending, K_FOREVER);
    int selected = -1;
    k_mutex_lock(&usb_tx_lock, K_FOREVER);
    for (uint8_t index = 0u; index < HEAD_USB_TX_QUEUE_DEPTH; ++index) {
      if (!usb_tx_queue[index].used) continue;
      if (selected < 0 ||
          (usb_tx_queue[index].priority && !usb_tx_queue[selected].priority) ||
          (usb_tx_queue[index].priority == usb_tx_queue[selected].priority &&
           usb_tx_queue[index].order < usb_tx_queue[selected].order)) {
        selected = index;
      }
    }
    uint16_t length = 0u;
    if (selected >= 0) {
      length = usb_tx_queue[selected].length;
      memcpy(frame, usb_tx_queue[selected].data, length);
      usb_tx_queue[selected].used = false;
    }
    k_mutex_unlock(&usb_tx_lock);
    if (length == 0u) continue;

    k_sem_reset(&usb_output_done);
    k_spinlock_key_t key = k_spin_lock(&usb_output.lock);
    memcpy(usb_output.frame, frame, length);
    usb_output.length = length;
    usb_output.offset = 0u;
    usb_output.result = -EINPROGRESS;
    usb_output.active = true;
    k_spin_unlock(&usb_output.lock, key);
    uart_irq_tx_enable(usb);
    const uint32_t output_deadline_ms = k_uptime_get_32() + 50u;
    while (usb_host_ready() &&
           (int32_t)(k_uptime_get_32() - output_deadline_ms) < 0) {
      if (k_sem_take(&usb_output_done, K_MSEC(5)) == 0) break;
    }
    key = k_spin_lock(&usb_output.lock);
    const uint16_t offset = usb_output.offset;
    const int output_result = usb_output.result;
    usb_output.active = false;
    uart_irq_tx_disable(usb);
    k_spin_unlock(&usb_output.lock, key);
    if (offset != length || output_result < 0) {
      atomic_inc(&usb_tx_dropped_frames);
      atomic_add(&usb_tx_dropped_bytes, length - offset);
      /* Let a receiver discard a truncated frame before trying fresh output. */
      k_sleep(K_MSEC(60));
    }
  }
}

static void send_ack(uint8_t for_type, int8_t result, uint32_t lease_token,
                     uint32_t transaction_id)
{
  uint8_t payload[14] = { for_type, (uint8_t)result };
  for (uint8_t byte_index = 0u; byte_index < 4u; ++byte_index) {
    payload[2u + byte_index] = (uint8_t)(lease_token >> (8u * byte_index));
    payload[6u + byte_index] = (uint8_t)(transaction_id >> (8u * byte_index));
    payload[10u + byte_index] = (uint8_t)(boot_session_id >> (8u * byte_index));
  }
  usb_send(result == 0 ? HEAD_MSG_ACK : HEAD_MSG_NACK, payload, sizeof(payload));
}

static void put_u16(uint8_t *buffer, size_t *byte_offset, uint16_t value)
{
  buffer[(*byte_offset)++] = (uint8_t)value;
  buffer[(*byte_offset)++] = (uint8_t)(value >> 8u);
}

static void put_u32(uint8_t *buffer, size_t *byte_offset, uint32_t value)
{
  for (uint8_t byte_index = 0; byte_index < 4u; ++byte_index) buffer[(*byte_offset)++] = (uint8_t)(value >> (8u * byte_index));
}

static void put_f32(uint8_t *buffer, size_t *byte_offset, float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  put_u32(buffer, byte_offset, bits);
}

static void send_hello_reply(uint32_t transaction_id)
{
  uint8_t payload[52];
  size_t byte_offset = 0u;
  uint64_t capabilities = HEAD_CAP_TRANSACTION_IDS |
      HEAD_CAP_BOUNDED_USB_TX | HEAD_CAP_TRANSMITTED_SEQUENCE |
      HEAD_CAP_SINGLE_SERVO_HOMING | HEAD_CAP_PARSER_TIMEOUT |
      HEAD_CAP_TORQUE_READBACK | HEAD_CAP_PROPRIOCEPTION_HOLD |
      HEAD_CAP_ROUTING_HOLD;
#if defined(CONFIG_NVS)
  capabilities |= HEAD_CAP_CONFIG_AB_STORAGE;
#endif
#if defined(CONFIG_WATCHDOG)
  capabilities |= HEAD_CAP_MCU_WATCHDOG;
#endif
  put_u32(payload, &byte_offset, transaction_id);
  put_u16(payload, &byte_offset, HEAD_HELLO_SCHEMA_VERSION);
  put_u16(payload, &byte_offset, HEAD_PROTOCOL_VERSION);
  payload[byte_offset++] = HEAD_FIRMWARE_VERSION_MAJOR;
  payload[byte_offset++] = HEAD_FIRMWARE_VERSION_MINOR;
  payload[byte_offset++] = HEAD_FIRMWARE_VERSION_PATCH;
  payload[byte_offset++] = 1u;
  put_u32(payload, &byte_offset, HEAD_PROTOCOL_SCHEMA_HASH);
  put_u32(payload, &byte_offset, (uint32_t)capabilities);
  put_u32(payload, &byte_offset, (uint32_t)(capabilities >> 32u));
  put_u32(payload, &byte_offset, HEAD_FIRMWARE_BUILD_ID);
  put_u32(payload, &byte_offset, HEAD_CONFIG_HARDWARE_ID);
  put_u32(payload, &byte_offset, boot_session_id);
  put_u32(payload, &byte_offset, boot_reset_cause);
  put_u32(payload, &byte_offset, head_config_generation());
  put_u16(payload, &byte_offset, HEAD_CONTROL_HZ);
  put_u16(payload, &byte_offset, HEAD_TELEMETRY_HZ);
  put_u16(payload, &byte_offset, CONFIG_USB_DEVICE_VID);
  put_u16(payload, &byte_offset, CONFIG_USB_DEVICE_PID);
  usb_send(HEAD_MSG_HELLO_REPLY, payload, (uint16_t)byte_offset);
}

static void encode_configuration_info(const struct head_calibration *source,
                                      uint8_t *payload, size_t *byte_offset)
{
  put_u32(payload, byte_offset, source->version);
  payload[(*byte_offset)++] = source->expected_servo_count;
  payload[(*byte_offset)++] = source->allow_partial_inventory;
  payload[(*byte_offset)++] = source->thermal_start_c;
  payload[(*byte_offset)++] = source->thermal_full_c;
  put_u32(payload, byte_offset, source->active_servo_mask);
}

static bool decode_configuration_info(const uint8_t *payload, uint16_t length,
                                      struct head_calibration *target)
{
  if (length != 12u) return false;
  target->version = get_u32_le(&payload[0]);
  target->expected_servo_count = payload[4];
  target->allow_partial_inventory = payload[5];
  target->thermal_start_c = payload[6];
  target->thermal_full_c = payload[7];
  target->active_servo_mask = get_u32_le(&payload[8]);
  return true;
}

static void encode_configuration_slot(uint8_t index, const struct head_joint_config *joint,
                                      uint8_t *payload, size_t *byte_offset)
{
  payload[(*byte_offset)++] = index;
  payload[(*byte_offset)++] = joint->branch_index;
  payload[(*byte_offset)++] = joint->servo_id;
  payload[(*byte_offset)++] = 0u;
  put_u32(payload, byte_offset, (uint32_t)joint->min_tick);
  put_u32(payload, byte_offset, (uint32_t)joint->max_tick);
  put_u32(payload, byte_offset, (uint32_t)joint->home_tick);
  put_f32(payload, byte_offset, joint->max_position_fraction_per_control_cycle);
  put_f32(payload, byte_offset, joint->max_acceleration_fraction_per_control_cycle_squared);
  put_f32(payload, byte_offset, joint->homing_direction);
  put_u32(payload, byte_offset, (uint32_t)joint->reserved_homing_start_tick);
  put_u32(payload, byte_offset, (uint32_t)joint->homing_max_travel_ticks);
  put_u32(payload, byte_offset, joint->homing_timeout_ms);
  put_u16(payload, byte_offset, (uint16_t)joint->operating_current_ma);
  put_u16(payload, byte_offset, (uint16_t)joint->homing_current_ma);
  put_u32(payload, byte_offset, (uint32_t)joint->homing_following_error_ticks);
  put_u16(payload, byte_offset, joint->homing_persistence_ms);
  put_u16(payload, byte_offset, (uint16_t)joint->homing_current_limit_ma);
  put_u16(payload, byte_offset, joint->homing_speed_ticks_per_second);
  put_u32(payload, byte_offset, (uint32_t)joint->homing_backoff_ticks);
}

static bool decode_configuration_slot(const uint8_t *payload, uint16_t length,
                                      uint8_t *index, struct head_joint_config *joint)
{
  size_t byte_offset = 0u;
  if (length != 58u) return false;
  memset(joint, 0, sizeof(*joint));
  *index = payload[byte_offset++];
  if (*index >= HEAD_SERVO_COUNT) return false;
  joint->branch_index = payload[byte_offset++];
  joint->servo_id = payload[byte_offset++];
  ++byte_offset;
  joint->min_tick = (int32_t)get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->max_tick = (int32_t)get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->home_tick = (int32_t)get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->max_position_fraction_per_control_cycle = get_f32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->max_acceleration_fraction_per_control_cycle_squared = get_f32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->homing_direction = get_f32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->reserved_homing_start_tick = (int32_t)get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->homing_max_travel_ticks = (int32_t)get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->homing_timeout_ms = get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->operating_current_ma = (int16_t)get_u16_le(&payload[byte_offset]); byte_offset += 2u;
  joint->homing_current_ma = (int16_t)get_u16_le(&payload[byte_offset]); byte_offset += 2u;
  joint->homing_following_error_ticks = (int32_t)get_u32_le(&payload[byte_offset]); byte_offset += 4u;
  joint->homing_persistence_ms = get_u16_le(&payload[byte_offset]); byte_offset += 2u;
  joint->homing_current_limit_ma = (int16_t)get_u16_le(&payload[byte_offset]); byte_offset += 2u;
  joint->homing_speed_ticks_per_second = get_u16_le(&payload[byte_offset]); byte_offset += 2u;
  joint->homing_backoff_ticks = (int32_t)get_u32_le(&payload[byte_offset]);
  return true;
}

static bool configuration_mutable(void)
{
  return !runtime.discovery_active && !runtime.diagnostic_active && !runtime.preparation_active &&
         runtime.storage_state != HEAD_STORAGE_PENDING &&
         runtime.storage_state != HEAD_STORAGE_WRITING &&
         (runtime.state == HEAD_HOMING_REQUIRED ||
          /* Staging is torque-off and the commit path still validates in full,
           * so a discovery failure must not lock the profile. A stored profile
           * that no longer matches the hardware faults on DISCOVERY, and
           * repairing it is exactly what an operator needs to do next; leaving
           * only CONFIGURATION mutable strands such a board, because clearing
           * the fault re-runs the discovery that cannot pass. */
          (runtime.state == HEAD_FAULT &&
           (runtime.fault == HEAD_FAULT_CONFIGURATION ||
            runtime.fault == HEAD_FAULT_DISCOVERY)));
}

static int queue_storage_locked(const struct head_calibration *source, bool apply)
{
  if (runtime.storage_state == HEAD_STORAGE_PENDING ||
      runtime.storage_state == HEAD_STORAGE_WRITING) return -EBUSY;
  storage_pending_calibration = *source;
  storage_pending_apply = apply;
  runtime.storage_state = HEAD_STORAGE_PENDING;
  runtime.storage_result = 0;
  k_sem_give(&storage_pending_sem);
  return 0;
}

static bool active_feedback_fresh(uint32_t now_ms)
{
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    if ((calibration.active_servo_mask & (1u << index)) == 0u) continue;
    const struct head_servo_state *servo = &runtime.servos[index];
    if (!servo->online || servo->last_feedback_ms == 0u ||
        now_ms - servo->last_feedback_ms > HEAD_COMMAND_FEEDBACK_FRESH_MS ||
        servo->hardware_error != 0u) return false;
  }
  return true;
}

static uint8_t active_branch_mask(void)
{
  uint8_t mask = 0u;
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (((calibration.active_servo_mask >> (branch_index * HEAD_SERVOS_PER_BRANCH)) & 0x1Fu) != 0u) {
      mask |= (uint8_t)(1u << branch_index);
    }
  }
  return mask;
}

static uint8_t next_active_servo(uint8_t start)
{
  for (uint8_t index = start; index < HEAD_SERVO_COUNT; ++index) {
    if ((calibration.active_servo_mask & (1u << index)) != 0u) return index;
  }
  return UINT8_MAX;
}

static void begin_preparation_locked(enum head_state target_state)
{
  head_dxl_abort_telemetry(&runtime);
  runtime.preparation_active = true;
  runtime.preparation_phase = 0u;
  runtime.preparation_servo_index = 0u;
  runtime.preparation_target_state = target_state;
  runtime.preparation_started_ms = k_uptime_get_32();
}

static void service_preparation_locked(void)
{
  const uint32_t started_ms = k_uptime_get_32();
  /* Branch current is the first motion-preparation safety gate. This check is
   * independent of config validation so a corrupted in-memory profile cannot
   * reach the torque-enable sequence. */
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    if (head_config_operating_current_branch_ma(&calibration, branch_index) >
        HEAD_BRANCH_CURRENT_BUDGET_MA) {
      head_state_fault(&runtime, HEAD_FAULT_BRANCH_CURRENT_BUDGET);
      return;
    }
  }
  if (head_lease_is_expired(&runtime, started_ms) ||
      started_ms - runtime.preparation_started_ms > 1500u) {
    head_state_fault(&runtime, HEAD_FAULT_WATCHDOG);
    return;
  }
  const int result = head_dxl_prepare_step(&runtime, &calibration);
  if (k_uptime_get_32() - started_ms > 20u) {
    head_state_fault(&runtime, HEAD_FAULT_CONTROL_DEADLINE);
  } else if (result < 0) {
    head_state_fault(&runtime, HEAD_FAULT_BUS);
  } else if (result == 1) {
    runtime.preparation_active = false;
    runtime.preparation_completed_ms = k_uptime_get_32();
    runtime.torque_state = HEAD_TORQUE_ON_VERIFIED;
    runtime.state = runtime.preparation_target_state;
    runtime.last_command_ms = k_uptime_get_32();
    runtime.proprioception_started_ms = runtime.last_command_ms;
    runtime.homing_torque_index = runtime.homing_index;
    runtime.homing_started_ms = 0u;
    runtime.accepted_sequence_valid = false;
    runtime.pending_transmit_valid = false;
    runtime.watchdog_hold_started_ms = 0u;
  }
}

static void service_branch_current_budget_locked(uint32_t now_ms)
{
  if (runtime.shutdown_requested || runtime.preparation_active ||
      !head_state_torque_may_be_on(&runtime)) {
    return;
  }

  /* Evaluate each branch as soon as that branch has a complete, fresh current
   * sample. A slow or absent response elsewhere must not delay over-current
   * shutdown for this branch. */
  for (uint8_t branch_index = 0u; branch_index < HEAD_BRANCH_COUNT;
       ++branch_index) {
    bool branch_current_feedback_fresh = true;
    const uint8_t first_servo = branch_index * HEAD_SERVOS_PER_BRANCH;
    const uint8_t end_servo = first_servo + HEAD_SERVOS_PER_BRANCH;
    for (uint8_t index = first_servo; index < end_servo; ++index) {
      if ((calibration.active_servo_mask & (1u << index)) == 0u) continue;
      const uint32_t feedback_ms = runtime.servos[index].last_current_feedback_ms;
      if (feedback_ms == 0u || now_ms - feedback_ms > 30u ||
          (int32_t)(feedback_ms - runtime.preparation_completed_ms) < 0) {
        branch_current_feedback_fresh = false;
        break;
      }
    }
    if (branch_current_feedback_fresh &&
        head_control_present_current_branch_ma(&runtime, &calibration,
                                               branch_index) >
            HEAD_BRANCH_CURRENT_BUDGET_MA) {
      head_state_fault(&runtime, HEAD_FAULT_BRANCH_CURRENT_BUDGET);
      return;
    }
  }
}

/* Must be called with head_lock held. Each sweep broadcasts torque-off and
 * then reads Torque Enable back from every configured servo.  Two verified
 * sweeps protect against a transient bus error during fault handling. */
static int service_shutdown_locked(uint32_t now_ms)
{
  int result;
  if (!runtime.shutdown_requested) return 0;
  if ((int32_t)(now_ms - runtime.shutdown_next_attempt_ms) < 0) return -EAGAIN;

  head_dxl_abort_telemetry(&runtime);
  (void)head_board_fan_set_percent(100u);
  result = head_dxl_set_torque_all(&calibration, false);
  if (head_dxl_take_hardware_alert_mask() != 0u) head_state_fault(&runtime, HEAD_FAULT_SERVO);
  if (head_board_release_all() != 0) result = -EIO;
  ++runtime.shutdown_attempts;
  if (result == 0) {
    runtime.torque_state = HEAD_TORQUE_SHUTDOWN_PENDING;
    if (runtime.shutdown_confirmations < 2u) ++runtime.shutdown_confirmations;
    if (runtime.shutdown_confirmations >= 2u) {
      runtime.shutdown_requested = false;
      runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
    } else {
      runtime.shutdown_next_attempt_ms = now_ms + 10u;
    }
  } else {
    runtime.shutdown_confirmations = 0u;
    ++runtime.shutdown_failures;
    runtime.torque_state = HEAD_TORQUE_SHUTDOWN_FAILED;
    const uint32_t shift = runtime.shutdown_failures > 6u ? 6u : runtime.shutdown_failures;
    runtime.shutdown_next_attempt_ms = now_ms + (10u << shift);
  }
  return result;
}

static void send_state(void)
{
  uint8_t payload[512];
  size_t byte_offset = 0u;
  uint32_t now;

  k_mutex_lock(&head_lock, K_FOREVER);
  now = k_uptime_get_32();
  payload[byte_offset++] = (uint8_t)runtime.state;
  payload[byte_offset++] = (uint8_t)runtime.fault;
  payload[byte_offset++] = head_state_torque_may_be_on(&runtime) ? 1u : 0u;
  payload[byte_offset++] = head_fan_stalled() ? 1u : 0u;
  put_u16(payload, &byte_offset, head_fan_rpm());
  put_u32(payload, &byte_offset, now);
  /* Protocol v2's legacy state slot is named applied_sequence by ROS. It now
   * reports the last command successfully transmitted on every active branch;
   * broadcast Sync Write cannot confirm physical application. */
  put_u32(payload, &byte_offset, runtime.last_transmitted_sequence);
  put_u16(payload, &byte_offset, HEAD_CONTROL_HZ);
  payload[byte_offset++] = (uint8_t)runtime.torque_state;
  payload[byte_offset++] = runtime.shutdown_confirmations;
  payload[byte_offset++] = runtime.shutdown_requested ? 1u : 0u;
  payload[byte_offset++] = 0u;
  put_u32(payload, &byte_offset, runtime.shutdown_attempts);
  put_u32(payload, &byte_offset, runtime.shutdown_failures);
  put_u32(payload, &byte_offset, boot_session_id);
  for (uint8_t servo_index = 0; servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    const struct head_servo_state *servo = &runtime.servos[servo_index];
    put_u32(payload, &byte_offset, (uint32_t)servo->goal_tick);
    put_u32(payload, &byte_offset, (uint32_t)servo->present_tick);
    put_u32(payload, &byte_offset, (uint32_t)servo->present_velocity_raw);
    put_u16(payload, &byte_offset, (uint16_t)servo->present_current_ma);
    put_u16(payload, &byte_offset, servo->present_voltage_mv);
    payload[byte_offset++] = servo->temperature_c;
    payload[byte_offset++] = servo->moving_status;
    payload[byte_offset++] = servo->hardware_error;
    payload[byte_offset++] = servo->online;
    put_u32(payload, &byte_offset, servo->last_feedback_ms == 0u ? UINT32_MAX :
                            now - servo->last_feedback_ms);
  }
  k_mutex_unlock(&head_lock);
  usb_send(HEAD_MSG_STATE, payload, (uint16_t)byte_offset);
}

static void send_diagnostics(void)
{
  uint8_t payload[224];
  size_t byte_offset = 0u;

  k_mutex_lock(&head_lock, K_FOREVER);
  payload[byte_offset++] = (uint8_t)runtime.state;
  payload[byte_offset++] = (uint8_t)runtime.fault;
  put_u32(payload, &byte_offset, runtime.control_deadline_misses);
  put_u32(payload, &byte_offset, runtime.last_control_cycle_ms);
  put_u32(payload, &byte_offset, runtime.control_max_period_us);
  for (uint8_t branch_index = 0; branch_index < HEAD_BRANCH_COUNT; ++branch_index) {
    const struct head_branch_health *health = &runtime.branches[branch_index];
    payload[byte_offset++] = branch_index;
    payload[byte_offset++] = health->telemetry_active;
    payload[byte_offset++] = health->telemetry_expected_mask;
    payload[byte_offset++] = health->telemetry_received_mask;
    put_u32(payload, &byte_offset, health->telemetry_requested_ms);
    put_u32(payload, &byte_offset, health->telemetry_completed_ms);
    put_u32(payload, &byte_offset, health->telemetry_timeouts);
    put_u32(payload, &byte_offset, health->protocol_errors);
    put_u32(payload, &byte_offset, health->bus_errors);
    put_u32(payload, &byte_offset, health->control_transmissions);
    put_u32(payload, &byte_offset, health->control_transmission_errors);
    put_u32(payload, &byte_offset, health->control_transmission_deferred);
  }
  put_u32(payload, &byte_offset, (uint32_t)atomic_get(&usb_tx_dropped_frames));
  put_u32(payload, &byte_offset, (uint32_t)atomic_get(&usb_tx_dropped_bytes));
  put_u32(payload, &byte_offset, (uint32_t)atomic_get(&usb_tx_coalesced_frames));
  payload[byte_offset++] = (uint8_t)runtime.storage_state;
  /* Two of the three reserved bytes carry the retried-transfer total, so the
   * bus loss rate the retries absorb stays observable without changing the
   * fixed diagnostics payload length. */
  put_u16(payload, &byte_offset, head_dxl_read_retry_count());
  payload[byte_offset++] = 0u;
  put_u32(payload, &byte_offset, (uint32_t)runtime.storage_result);
  put_u32(payload, &byte_offset, head_config_generation());
  put_u32(payload, &byte_offset, boot_reset_cause);
  /* Per-servo discovery rejection reason, so a -ENODEV names the check. */
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    payload[byte_offset++] = head_dxl_discovery_reason(index);
  }
  uint16_t failed_address = 0u;
  uint8_t failed_status = 0u;
  head_dxl_last_transaction_error(&failed_address, &failed_status);
  put_u16(payload, &byte_offset, failed_address);
  payload[byte_offset++] = failed_status;
  k_mutex_unlock(&head_lock);
  usb_send(HEAD_MSG_DIAGNOSTICS, payload, (uint16_t)byte_offset);
}

static void send_probe_branch_result(uint8_t branch_index,
                                     const struct head_dxl_probe_reply *replies,
                                     size_t reply_count)
{
  uint8_t payload[2u + HEAD_SERVOS_PER_BRANCH * 4u];
  size_t byte_offset = 0u;

  payload[byte_offset++] = branch_index;
  payload[byte_offset++] = (uint8_t)reply_count;
  for (size_t servo_index = 0u; servo_index < reply_count; ++servo_index) {
    payload[byte_offset++] = replies[servo_index].servo_id;
    put_u16(payload, &byte_offset, replies[servo_index].model_number);
    payload[byte_offset++] = replies[servo_index].firmware_version;
  }
  usb_send(HEAD_MSG_PROBE_BRANCH_RESULT, payload, (uint16_t)byte_offset);
}

static void send_rx_line_test_result(uint8_t branch_index, uint32_t bytes, uint32_t errors)
{
  uint8_t payload[9];
  size_t byte_offset = 0u;
  payload[byte_offset++] = branch_index;
  put_u32(payload, &byte_offset, bytes);
  put_u32(payload, &byte_offset, errors);
  usb_send(HEAD_MSG_UART_RX_LINE_TEST_RESULT, payload, sizeof(payload));
}

static void send_debug_ping_result(uint8_t branch_index, uint8_t servo_id,
                                   const struct head_dxl_debug_ping *diagnostic)
{
  uint8_t payload[13u + sizeof(diagnostic->request) + sizeof(diagnostic->received) + 8u];
  size_t byte_offset = 0u;
  payload[byte_offset++] = branch_index;
  payload[byte_offset++] = servo_id;
  payload[byte_offset++] = diagnostic->flags;
  payload[byte_offset++] = diagnostic->request_length;
  payload[byte_offset++] = diagnostic->received_length;
  put_u32(payload, &byte_offset, diagnostic->first_byte_delay_us);
  put_u32(payload, &byte_offset, diagnostic->uart_error_flags);
  memcpy(&payload[byte_offset], diagnostic->request, diagnostic->request_length);
  byte_offset += diagnostic->request_length;
  memcpy(&payload[byte_offset], diagnostic->received, diagnostic->received_length);
  byte_offset += diagnostic->received_length;
  /* Forensics trail after the captured bytes so the fixed header offsets the
   * host already parses stay valid. */
  put_u16(payload, &byte_offset, diagnostic->stale_bytes);
  put_u16(payload, &byte_offset, diagnostic->rx_disabled_count);
  put_u16(payload, &byte_offset, diagnostic->rx_restart_failures);
  put_u16(payload, &byte_offset, diagnostic->rx_overflow_count);
  usb_send(HEAD_MSG_DEBUG_PING_RESULT, payload, (uint16_t)byte_offset);
}

static bool probe_baud_supported(uint32_t baudrate)
{
  return baudrate == 57600u || baudrate == 115200u || baudrate == 1000000u ||
         baudrate == 2000000u || baudrate == 3000000u || baudrate == 4000000u;
}

static void dispatch_frame(const struct head_frame *frame)
{
  struct head_command command;
  uint32_t now = k_uptime_get_32();
  int result = -EPERM;
  uint32_t transaction_id = 0u;
  bool defer_ack = false;

  if (frame->type != HEAD_MSG_JOINT_TARGETS) {
    if (frame->length < sizeof(transaction_id)) {
      send_ack(frame->type, -EINVAL, 0u, 0u);
      return;
    }
    transaction_id = get_u32_le(
        &frame->payload[frame->length - sizeof(transaction_id)]);
  }

  k_mutex_lock(&head_lock, K_FOREVER);
  if (atomic_get(&boot_complete) == 0 && frame->type != HEAD_MSG_HELLO) {
    result = -EAGAIN;
  } else switch (frame->type) {
  case HEAD_MSG_HELLO:
    if (frame->length == 4u) {
      send_hello_reply(transaction_id);
      result = 0;
    } else result = -EINVAL;
    break;
  case HEAD_MSG_ACQUIRE_LEASE:
    if (frame->length >= 4u && head_lease_is_expired(&runtime, now)) {
      ++lease_nonce;
      runtime.active_lease_token = boot_session_id ^ (lease_nonce * 0x9E3779B9u);
      if (runtime.active_lease_token == 0u) runtime.active_lease_token = 1u;
      head_lease_renew(&runtime, now);
      runtime.accepted_sequence_valid = false;
      result = 0;
    }
    break;
  case HEAD_MSG_RENEW_LEASE:
    if (frame->length == 8u && token_matches(frame, now)) {
      head_lease_renew(&runtime, now);
      result = 0;
    }
    break;
  case HEAD_MSG_RELEASE_LEASE:
    if (frame->length == 8u && token_matches(frame, now)) {
      if (runtime.diagnostic_active || runtime.preparation_active || head_state_torque_may_be_on(&runtime)) {
        head_state_disable(&runtime);
        result = service_shutdown_locked(now);
      } else {
        runtime.active_lease_token = 0u;
        runtime.accepted_sequence_valid = false;
        result = 0;
      }
    }
    break;
  case HEAD_MSG_JOINT_TARGETS:
    if (head_protocol_decode_command(frame, &command) == 0) {
      result = head_control_accept_command(&runtime, &command, &calibration, now) ? 0 : -EPERM;
      if (result == 0) head_lease_renew(&runtime, now);
    } else result = -EINVAL;
    break;
  case HEAD_MSG_HOME:
    if (frame->length == 8u && token_matches(frame, now) && head_state_can_prepare_motion(&runtime) && active_feedback_fresh(now) &&
      head_state_request_home(&runtime, &calibration)) {
      runtime.homing_index = next_active_servo(0u);
      runtime.homing_torque_index = UINT8_MAX;
      begin_preparation_locked(runtime.state);
      result = 0;
    }
    break;
  case HEAD_MSG_ZERO_HOME:
    if (frame->length == 8u && token_matches(frame, now) && head_state_can_prepare_motion(&runtime) && active_feedback_fresh(now) &&
      head_state_request_zero_home(&runtime, &calibration)) {
      runtime.homing_index = next_active_servo(0u);
      runtime.homing_torque_index = UINT8_MAX;
      begin_preparation_locked(runtime.state);
      result = 0;
    }
    break;
  case HEAD_MSG_MAINTENANCE_CALIBRATE:
    if (frame->length == 8u && token_matches(frame, now) && head_state_can_prepare_motion(&runtime) && active_feedback_fresh(now) &&
      head_state_request_maintenance_calibration(&runtime, &calibration)) {
      runtime.homing_index = next_active_servo(0u);
      runtime.homing_torque_index = UINT8_MAX;
      begin_preparation_locked(runtime.state);
      result = 0;
    }
    break;
  case HEAD_MSG_CALIBRATION_CONFIRM:
    if (token_matches(frame, now) && frame->length == 9u &&
        head_state_confirm_maintenance_servo(&runtime, &calibration, frame->payload[4])) {
      result = 0;
    }
    break;
  case HEAD_MSG_ENABLE:
    if (frame->length == 8u && token_matches(frame, now) &&
        runtime.state == HEAD_READY &&
        runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED &&
        head_state_can_prepare_motion(&runtime) && active_feedback_fresh(now)) {
      begin_preparation_locked(HEAD_ENABLED);
      result = 0;
    }
    break;
  case HEAD_MSG_START_PROPRIOCEPTION:
    if (frame->length == 8u && token_matches(frame, now) &&
        runtime.state == HEAD_READY &&
        runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED &&
        head_state_can_prepare_motion(&runtime) && active_feedback_fresh(now)) {
      begin_preparation_locked(HEAD_PROPRIOCEPTION_SETTLING);
      result = 0;
    }
    break;
  case HEAD_MSG_START_ROUTING:
    if (frame->length == 8u && token_matches(frame, now) &&
        (runtime.state == HEAD_HOMING_REQUIRED || runtime.state == HEAD_READY) &&
        runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED &&
        head_state_can_prepare_motion(&runtime) && active_feedback_fresh(now)) {
      bool target_valid = true;
      for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
        if ((calibration.active_servo_mask & (1u << index)) == 0u) continue;
        const struct head_joint_config *joint = &calibration.joints[index];
        if (HEAD_ROUTING_BASE_TICK < joint->min_tick ||
            HEAD_ROUTING_BASE_TICK > joint->max_tick) {
          target_valid = false;
          break;
        }
        /* Seed the torque-on write to measured position. The control loop
         * subsequently ramps each goal to zero through the normal limiter. */
        runtime.servos[index].goal_tick = runtime.servos[index].present_tick;
        runtime.servos[index].goal_step_ticks_per_control_cycle = 0;
      }
      if (target_valid) {
        begin_preparation_locked(HEAD_ROUTING);
        result = 0;
      } else {
        result = -ERANGE;
      }
    }
    break;
  case HEAD_MSG_DISABLE:
    if (frame->length == 8u && token_matches(frame, now)) {
      head_state_disable(&runtime);
      result = service_shutdown_locked(now);
    }
    break;
  case HEAD_MSG_CLEAR_FAULT: {
    /* A discovery failure on a silent bus parks the shutdown sweep in
     * SHUTDOWN_FAILED and leaves shutdown_requested set forever, so the
     * ordinary OFF_VERIFIED precondition can never be met and a power cycle
     * returns to the identical state. Admit the retry when torque was never
     * commanded on; the sweep's own verification is left untouched. */
    const bool never_energized = head_state_torque_never_commanded_on(&runtime);
    if (frame->length == 8u && token_matches(frame, now) && runtime.state == HEAD_FAULT &&
        (runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED || never_energized) &&
        (!runtime.shutdown_requested || never_energized) &&
        !runtime.discovery_active && !runtime.preparation_active && !runtime.diagnostic_active &&
        runtime.storage_state != HEAD_STORAGE_PENDING && runtime.storage_state != HEAD_STORAGE_WRITING) {
      if (!head_config_validate(&calibration)) {
        result = -EINVAL;
      } else {
        /* discover_inventory_locked() aborts with -ECANCELED while a shutdown
         * is outstanding, and it opens with its own broadcast Torque Enable=0,
         * so retract the stuck request rather than race it. The discovery
         * thread reinstates it if the sweep fails again. */
        runtime.shutdown_requested = false;
        runtime.shutdown_confirmations = 0u;
        runtime.shutdown_next_attempt_ms = 0u;
        runtime.discovery_active = true;
        runtime.discovery_verified = false;
        discovery_transaction_id = transaction_id;
        k_sem_give(&discovery_pending_sem);
        defer_ack = true;
        result = 0;
      }
    }
    break;
  }
  case HEAD_MSG_FAN_OVERRIDE:
    if (token_matches(frame, now) && frame->length == 9u) {
      runtime.fan_override = frame->payload[4] <= 100u;
      runtime.fan_override_percent = frame->payload[4];
      result = runtime.fan_override ? 0 : -EINVAL;
    }
    break;
  case HEAD_MSG_GET_CONFIGURATION_INFO: {
    uint8_t payload[12];
    size_t byte_offset = 0u;
    if (frame->length == 4u) {
      encode_configuration_info(&calibration, payload, &byte_offset);
      usb_send(HEAD_MSG_CONFIGURATION_INFO, payload, (uint16_t)byte_offset);
      result = 0;
    } else result = -EINVAL;
    break;
  }
  case HEAD_MSG_GET_CONFIGURATION_SLOT: {
    uint8_t payload[58];
    size_t byte_offset = 0u;
    if (frame->length == 5u && frame->payload[0] < HEAD_SERVO_COUNT) {
      encode_configuration_slot(frame->payload[0], &calibration.joints[frame->payload[0]],
                                payload, &byte_offset);
      usb_send(HEAD_MSG_CONFIGURATION_SLOT, payload, (uint16_t)byte_offset);
      result = 0;
    } else result = -EINVAL;
    break;
  }
  case HEAD_MSG_STAGE_CONFIGURATION_INFO:
    if (token_matches(frame, now) && configuration_mutable() && frame->length == 20u) {
      head_config_default(&staged_calibration);
      staging_active = decode_configuration_info(&frame->payload[4], 12u,
                                                  &staged_calibration);
      staged_slots_mask = 0u;
      result = staging_active ? 0 : -EINVAL;
    }
    break;
  case HEAD_MSG_STAGE_CONFIGURATION_SLOT: {
    uint8_t index;
    struct head_joint_config joint;
    if (token_matches(frame, now) && configuration_mutable() && staging_active &&
        frame->length == 66u &&
        decode_configuration_slot(&frame->payload[4], 58u, &index, &joint) &&
        (staged_calibration.active_servo_mask & (1u << index)) != 0u) {
      staged_calibration.joints[index] = joint;
      staged_slots_mask |= 1u << index;
      result = 0;
    }
    break;
  }
  case HEAD_MSG_COMMIT_CONFIGURATION:
    if (frame->length == 8u && token_matches(frame, now) &&
        configuration_mutable() && staging_active &&
        staged_slots_mask == staged_calibration.active_servo_mask) {
      head_config_finalize(&staged_calibration);
      if (head_config_validate(&staged_calibration)) {
        result = queue_storage_locked(&staged_calibration, true);
        if (result == 0) {
        staging_active = false;
        }
      } else result = -EINVAL;
    }
    break;
  case HEAD_MSG_PROBE_BRANCH: {
    struct head_dxl_probe_reply replies[HEAD_SERVOS_PER_BRANCH];
    size_t reply_count = 0u;
    uint8_t branch_index = HEAD_BRANCH_COUNT;
    uint32_t baudrate = HEAD_BRANCH_DEFAULT_BAUD;
    if (frame->length == 9u) {
      branch_index = frame->payload[0];
      baudrate = get_u32_le(&frame->payload[1]);
    }

    /* Do not interleave a diagnostic read with homing or motion. The branch
     * is restored to the production default rate before replying. */
    if (branch_index < HEAD_BRANCH_COUNT && probe_baud_supported(baudrate) &&
        !runtime.discovery_active && !runtime.diagnostic_active && !runtime.preparation_active &&
        (!head_state_torque_may_be_on(&runtime) ||
         head_state_torque_never_commanded_on(&runtime)) &&
        runtime.state != HEAD_HOMING &&
        runtime.state != HEAD_MAINTENANCE_CALIBRATION &&
        runtime.state != HEAD_ENABLED) {
      /* Telemetry Sync Read keeps polling every branch even in HEAD_READY.
       * A reply can still be in flight on the wire; stop and drain it first
       * so this diagnostic never collides with it. */
      head_dxl_abort_telemetry(&runtime);
      result = head_board_branch_set_baud(branch_index, baudrate);
      if (result == 0) {
        result = head_dxl_probe_branch(branch_index, replies, HEAD_SERVOS_PER_BRANCH,
                                       &reply_count);
        const int restore_result = head_board_branch_set_baud(
            branch_index, HEAD_BRANCH_DEFAULT_BAUD);
        if (result == 0 && restore_result != 0) result = restore_result;
      }
      if (result == 0) send_probe_branch_result(branch_index, replies, reply_count);
    }
    break;
  }
  case HEAD_MSG_DEBUG_PING: {
    struct head_dxl_debug_ping diagnostic;
    uint8_t branch_index = HEAD_BRANCH_COUNT;
    uint8_t servo_id = DXL_BROADCAST_ID;
    uint32_t baudrate = 1000000u;
    if (frame->length == 10u) {
      branch_index = frame->payload[0];
      servo_id = frame->payload[1];
      baudrate = get_u32_le(&frame->payload[2]);
    }
    if (branch_index < HEAD_BRANCH_COUNT && servo_id < DXL_BROADCAST_ID &&
        probe_baud_supported(baudrate) && !runtime.discovery_active && !runtime.diagnostic_active &&
        !runtime.preparation_active &&
        (!head_state_torque_may_be_on(&runtime) ||
         head_state_torque_never_commanded_on(&runtime)) &&
        runtime.state != HEAD_HOMING &&
        runtime.state != HEAD_MAINTENANCE_CALIBRATION &&
        runtime.state != HEAD_ENABLED) {
      /* See HEAD_MSG_PROBE_BRANCH: a telemetry reply may still be inbound on
       * this branch even in HEAD_READY. Stop and drain it before taking the
       * bus, or the ping request/response can collide with it on the wire. */
      head_dxl_abort_telemetry(&runtime);
      result = head_board_branch_set_baud(branch_index, baudrate);
      if (result == 0) {
        result = head_dxl_debug_ping(branch_index, servo_id, &diagnostic);
        const int restore_result = head_board_branch_set_baud(
            branch_index, HEAD_BRANCH_DEFAULT_BAUD);
        if (result == 0 && restore_result != 0) result = restore_result;
      }
      if (result == 0) send_debug_ping_result(branch_index, servo_id, &diagnostic);
    }
    break;
  }
  case HEAD_MSG_DEBUG_READ: {
    uint8_t data[32];
    uint8_t branch_index = HEAD_BRANCH_COUNT;
    uint8_t servo_id = DXL_BROADCAST_ID;
    uint16_t address = 0u;
    uint8_t length = 0u;
    if (frame->length == 9u) {
      branch_index = frame->payload[0];
      servo_id = frame->payload[1];
      address = (uint16_t)frame->payload[2] | ((uint16_t)frame->payload[3] << 8u);
      length = frame->payload[4];
    }
    if (branch_index < HEAD_BRANCH_COUNT && servo_id < DXL_BROADCAST_ID &&
        length != 0u && length <= sizeof(data) && !runtime.discovery_active &&
        !runtime.diagnostic_active && !runtime.preparation_active &&
        (!head_state_torque_may_be_on(&runtime) ||
         head_state_torque_never_commanded_on(&runtime)) &&
        runtime.state != HEAD_HOMING &&
        runtime.state != HEAD_MAINTENANCE_CALIBRATION &&
        runtime.state != HEAD_ENABLED) {
      head_dxl_abort_telemetry(&runtime);
      result = head_dxl_debug_read(branch_index, servo_id, address, length, data);
      if (result == 0) {
        uint8_t payload[6u + sizeof(data)];
        size_t byte_offset = 0u;
        payload[byte_offset++] = branch_index;
        payload[byte_offset++] = servo_id;
        put_u16(payload, &byte_offset, address);
        payload[byte_offset++] = length;
        memcpy(&payload[byte_offset], data, length);
        byte_offset += length;
        usb_send(HEAD_MSG_DEBUG_READ_RESULT, payload, (uint16_t)byte_offset);
      }
    }
    break;
  }
  case HEAD_MSG_LINE_MODE_TEST:
  case HEAD_MSG_UART_TX_METER_TEST:
  case HEAD_MSG_UART_RX_LINE_TEST: {
    const bool line_mode = frame->type == HEAD_MSG_LINE_MODE_TEST;
    /* These three diagnostics are the tools for finding out why a branch is
     * silent, but HEAD_TORQUE_OFF_VERIFIED is only reachable through a
     * successful discovery -- so a dead branch locks the very test that would
     * diagnose it, and a branch that never answers can never be measured.
     * Under the bench flag, admit the same never-energized retry that
     * HEAD_MSG_CLEAR_FAULT and profile staging already allow for the identical
     * reason. All three are torque-off by construction: line-test only drives
     * the direction pin, the TX meter sends a torque-safe zero-byte stream,
     * and the RX test only listens. */
#if defined(HEAD_BENCH_LINE_TEST_UNGATED)
    const bool diagnostic_never_energized = head_state_torque_never_commanded_on(&runtime);
#else
    const bool diagnostic_never_energized = false;
#endif
    if (frame->length == (line_mode ? 6u : 5u) &&
        frame->payload[0] < HEAD_BRANCH_COUNT &&
        (!line_mode || frame->payload[1] <= 1u) &&
        (runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED || diagnostic_never_energized) &&
        (!runtime.shutdown_requested || diagnostic_never_energized) &&
        !runtime.discovery_active && !runtime.preparation_active &&
        !runtime.diagnostic_active &&
        runtime.storage_state != HEAD_STORAGE_PENDING &&
        runtime.storage_state != HEAD_STORAGE_WRITING) {
      head_dxl_abort_telemetry(&runtime);
      pending_diagnostic.message_type = frame->type;
      pending_diagnostic.branch_index = frame->payload[0];
      pending_diagnostic.transmit_enabled = line_mode ? frame->payload[1] : 0u;
      pending_diagnostic.transaction_id = transaction_id;
      runtime.diagnostic_cancel_requested = false;
      runtime.diagnostic_active = true;
      k_sem_give(&diagnostic_pending_sem);
      defer_ack = true;
      result = 0;
    }
    break;
  }
  default:
    result = -EINVAL;
    break;
  }
  const uint32_t ack_lease_token = runtime.active_lease_token;
  k_mutex_unlock(&head_lock);
  if (frame->type != HEAD_MSG_JOINT_TARGETS && !defer_ack) {
    send_ack(frame->type, (int8_t)result, ack_lease_token, transaction_id);
  }
}

/* Bus reservation persists between short slices, but the control supervisor
 * and host dispatcher run throughout the ten-second electrical measurement. */
static void diagnostic_thread(void *first, void *second, void *third)
{
  ARG_UNUSED(first); ARG_UNUSED(second); ARG_UNUSED(third);
  while (true) {
    k_sem_take(&diagnostic_pending_sem, K_FOREVER);
    const struct diagnostic_request request = pending_diagnostic;
    const uint32_t deadline_ms = k_uptime_get_32() + 10000u;
    uint32_t received_bytes = 0u;
    uint32_t error_flags = 0u;
    int result = 0;
    k_mutex_lock(&head_lock, K_FOREVER);
    if (request.message_type == HEAD_MSG_UART_TX_METER_TEST) {
      result = head_board_branch_set_baud(request.branch_index, 1000000u);
    }
    k_mutex_unlock(&head_lock);
    do {
      k_mutex_lock(&head_lock, K_FOREVER);
      if (runtime.diagnostic_cancel_requested || runtime.shutdown_requested ||
          runtime.torque_state != HEAD_TORQUE_OFF_VERIFIED) {
        result = -ECANCELED;
      } else if (result == 0 && request.message_type == HEAD_MSG_LINE_MODE_TEST) {
        result = head_board_branch_set_tx(request.branch_index, request.transmit_enabled != 0u);
      } else if (result == 0 && request.message_type == HEAD_MSG_UART_TX_METER_TEST) {
        result = head_board_branch_uart_tx_meter_test(request.branch_index, 1u);
      } else if (result == 0) {
        uint8_t incoming[128];
        received_bytes += head_board_branch_read_available(request.branch_index, incoming, sizeof(incoming));
        error_flags |= head_board_branch_error_flags(request.branch_index);
      }
      k_mutex_unlock(&head_lock);
      if (result != 0 || (request.message_type == HEAD_MSG_LINE_MODE_TEST &&
                          request.transmit_enabled == 0u)) break;
      k_sleep(K_MSEC(1));
    } while ((int32_t)(k_uptime_get_32() - deadline_ms) < 0);
    k_mutex_lock(&head_lock, K_FOREVER);
    if (head_board_branch_set_tx(request.branch_index, false) != 0) result = -EIO;
    if (request.message_type == HEAD_MSG_UART_TX_METER_TEST &&
        head_board_branch_set_baud(request.branch_index, HEAD_BRANCH_DEFAULT_BAUD) != 0) result = -EIO;
    runtime.diagnostic_active = false;
    const uint32_t lease_token = runtime.active_lease_token;
    k_mutex_unlock(&head_lock);
    if (result == 0 && request.message_type == HEAD_MSG_UART_RX_LINE_TEST) {
      send_rx_line_test_result(request.branch_index, received_bytes, error_flags);
    }
    send_ack(request.message_type, (int8_t)result, lease_token, request.transaction_id);
  }
}

static void control_thread(void *a, void *b, void *c)
{
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  int64_t next_tick = k_uptime_ticks();
  uint64_t last_cycle = 0u;
  const int64_t period_ticks = k_ms_to_ticks_ceil64(1000u / HEAD_CONTROL_HZ);
  while (true) {
    bool homing_reconfigured = false;
    next_tick += period_ticks;
    k_mutex_lock(&head_lock, K_FOREVER);
    const bool maintenance_was_running = runtime.state == HEAD_MAINTENANCE_CALIBRATION;
    const uint32_t now = k_uptime_get_32();
    const uint64_t cycle = k_cycle_get_64();
    uint32_t control_period_us = 0u;
    if (last_cycle != 0u) {
      control_period_us = (uint32_t)k_cyc_to_us_floor64(cycle - last_cycle);
      if (control_period_us > runtime.control_max_period_us) {
        runtime.control_max_period_us = control_period_us;
      }
      if (control_period_us > HEAD_CONTROL_PERIOD_US) {
        runtime.control_deadline_misses +=
            (control_period_us - 1u) / HEAD_CONTROL_PERIOD_US;
      }
    }
    last_cycle = cycle;
    if (!runtime.shutdown_requested && head_state_torque_may_be_on(&runtime) && head_lease_is_expired(&runtime, now)) {
      head_state_fault(&runtime, HEAD_FAULT_WATCHDOG);
    }
    if (!runtime.shutdown_requested && !runtime.preparation_active &&
        head_state_torque_may_be_on(&runtime) &&
        control_period_us > HEAD_CONTROL_FATAL_DELAY_US) {
      head_state_fault(&runtime, HEAD_FAULT_CONTROL_DEADLINE);
    }
    runtime.last_control_cycle_ms = now;
    if (runtime.preparation_active) {
      if (control_period_us > 20000u) head_state_fault(&runtime, HEAD_FAULT_CONTROL_DEADLINE);
      if (runtime.preparation_active) service_preparation_locked();
      homing_reconfigured = !runtime.preparation_active;
      goto supervise_control;
    }
    if (runtime.diagnostic_active || runtime.discovery_active ||
        runtime.storage_state == HEAD_STORAGE_WRITING) goto supervise_control;
    if (!runtime.diagnostic_active &&
        (runtime.state == HEAD_HOMING_REQUIRED || runtime.state == HEAD_HOMING ||
        runtime.state == HEAD_MAINTENANCE_CALIBRATION || runtime.state == HEAD_READY ||
        runtime.state == HEAD_ENABLED ||
        runtime.state == HEAD_PROPRIOCEPTION_SETTLING ||
        runtime.state == HEAD_PROPRIOCEPTION_HOLD ||
        runtime.state == HEAD_ROUTING)) {
      /* Consume responses before control writes. Starting the next telemetry
       * round happens below, after all branch writes have completed. */
      head_dxl_telemetry_tick(&runtime, &calibration, now, false);
    }
    head_state_homing_tick(&runtime, &calibration, now);
    head_state_proprioception_tick(&runtime, now);
    if ((runtime.state == HEAD_HOMING ||
         runtime.state == HEAD_MAINTENANCE_CALIBRATION) &&
        runtime.homing_index != runtime.homing_torque_index) {
      runtime.homing_index = next_active_servo(runtime.homing_index);
      if (runtime.homing_index >= HEAD_SERVO_COUNT) {
        head_state_fault(&runtime, HEAD_FAULT_HOMING);
      } else {
        begin_preparation_locked(runtime.state);
        homing_reconfigured = true;
        goto supervise_control;
      }
    }
    if (maintenance_was_running && runtime.state == HEAD_READY &&
        queue_storage_locked(&calibration, false) != 0) {
      head_state_fault(&runtime, HEAD_FAULT_CONFIGURATION);
    }
    head_control_tick(&runtime, &calibration, now);
    if (head_state_torque_verified_on(&runtime) && (runtime.state == HEAD_HOMING ||
        runtime.state == HEAD_MAINTENANCE_CALIBRATION || runtime.state == HEAD_ENABLED ||
        runtime.state == HEAD_PROPRIOCEPTION_SETTLING ||
        runtime.state == HEAD_PROPRIOCEPTION_HOLD ||
        runtime.state == HEAD_ROUTING)) {
      uint8_t written_branch_mask = 0u;
      if (runtime.state == HEAD_ENABLED && runtime.accepted_sequence_valid &&
          (!runtime.pending_transmit_valid ||
           runtime.pending_transmit_sequence != runtime.last_accepted_sequence)) {
        runtime.pending_transmit_sequence = runtime.last_accepted_sequence;
        runtime.pending_transmit_branch_mask = active_branch_mask();
        runtime.pending_transmit_valid = true;
      }
      if (head_dxl_write_targets(&runtime, &calibration, &written_branch_mask) != 0) {
        head_state_fault(&runtime, HEAD_FAULT_BUS);
      } else if (runtime.pending_transmit_valid) {
        runtime.pending_transmit_branch_mask &= (uint8_t)~written_branch_mask;
        if (runtime.pending_transmit_branch_mask == 0u) {
          runtime.last_transmitted_sequence = runtime.pending_transmit_sequence;
          runtime.pending_transmit_valid = false;
        }
      }
    }
    if (runtime.state == HEAD_HOMING_REQUIRED || runtime.state == HEAD_HOMING ||
        runtime.state == HEAD_MAINTENANCE_CALIBRATION || runtime.state == HEAD_READY ||
        runtime.state == HEAD_ENABLED ||
        runtime.state == HEAD_PROPRIOCEPTION_SETTLING ||
        runtime.state == HEAD_PROPRIOCEPTION_HOLD ||
        runtime.state == HEAD_ROUTING) {
      head_dxl_telemetry_tick(&runtime, &calibration, k_uptime_get_32(), true);
    }
supervise_control:
    (void)service_shutdown_locked(k_uptime_get_32());
#if defined(CONFIG_WATCHDOG)
    if (hardware_watchdog_channel >= 0 &&
        wdt_feed(hardware_watchdog, hardware_watchdog_channel) != 0) {
      head_state_fault(&runtime, HEAD_FAULT_WATCHDOG);
      hardware_watchdog_channel = -1;
    }
#endif
    k_mutex_unlock(&head_lock);
    if (homing_reconfigured) {
      /* Register readback intentionally exceeds a normal 2 ms cycle. Do not
       * report that commissioning operation as an execution deadline miss. */
      last_cycle = 0u;
      next_tick = k_uptime_ticks();
    }
    if (next_tick < k_uptime_ticks()) next_tick = k_uptime_ticks() + period_ticks;
    k_sleep(K_TIMEOUT_ABS_TICKS(next_tick));
  }
}

static void telemetry_thread(void *a, void *b, void *c)
{
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  uint32_t last_fan_ms = 0u;
  while (true) {
    k_mutex_lock(&head_lock, K_FOREVER);
    if (runtime.state == HEAD_HOMING_REQUIRED || runtime.state == HEAD_HOMING ||
        runtime.state == HEAD_MAINTENANCE_CALIBRATION ||
        runtime.state == HEAD_READY || runtime.state == HEAD_ENABLED ||
        runtime.state == HEAD_PROPRIOCEPTION_SETTLING ||
        runtime.state == HEAD_PROPRIOCEPTION_HOLD ||
        runtime.state == HEAD_ROUTING) {
      const uint32_t now = k_uptime_get_32();
      service_branch_current_budget_locked(now);
      if (!runtime.shutdown_requested && !runtime.preparation_active &&
          now - runtime.preparation_completed_ms > 30u && head_state_torque_may_be_on(&runtime)) {
        for (uint8_t index = 0; index < HEAD_SERVO_COUNT; ++index) {
          if ((calibration.active_servo_mask & (1u << index)) == 0u) continue;
          /* Age alone, not servo->online: telemetry clears online after a
           * single timed-out Sync Read and restores it on the next good one,
           * so online flaps on any lost reply while the feedback driving the
           * servos is still current. Age is what actually says whether torque
           * is being held on stale data. */
          if (now - runtime.servos[index].last_feedback_ms >
              HEAD_TELEMETRY_STALE_FAULT_MS) {
            head_state_fault(&runtime, HEAD_FAULT_TELEMETRY);
            break;
          }
          if (runtime.servos[index].hardware_error != 0u) {
            head_state_fault(&runtime, HEAD_FAULT_SERVO);
            break;
          }
        }
      }
    }
    if (k_uptime_get_32() - last_fan_ms >= HEAD_TELEMETRY_PERIOD_MS) {
      const uint32_t now = k_uptime_get_32();
      const int fan_result = head_fan_tick(&runtime, &calibration, now);
#if defined(HEAD_BENCH_NO_FAN)
      /* Bench image: no fan is wired, so the tach can never report RPM and the
       * stall detector would latch a fault the moment torque is applied. The
       * thermal duty cycle is still computed and driven; only the shutdown is
       * suppressed. Servo Hardware Error Status still faults on overheating. */
      ARG_UNUSED(fan_result);
#else
      if (head_state_torque_may_be_on(&runtime) &&
          (fan_result != 0 || head_fan_stalled())) {
        head_state_fault(&runtime, HEAD_FAULT_FAN);
      }
#endif
      last_fan_ms = now;
    }
    k_mutex_unlock(&head_lock);
    k_sleep(K_MSEC(1));
  }
}

/* Enter/return holding head_lock. Reservation excludes all other bus users;
 * releasing the lock between individual servo checks allows control health
 * checkpoints and host cancellation throughout recovery. No torque-on occurs. */
static int discover_inventory_locked(void)
{
  runtime.discovery_active = true;
  head_dxl_abort_telemetry(&runtime);
  int result = head_dxl_emergency_torque_off();
  for (uint8_t servo_index = 0u; result == 0 && servo_index < HEAD_SERVO_COUNT; ++servo_index) {
    if ((calibration.active_servo_mask & (1u << servo_index)) == 0u) continue;
    if (runtime.shutdown_requested) { result = -ECANCELED; break; }
    const uint32_t started_ms = k_uptime_get_32();
    result = head_dxl_discover_one(&runtime, &calibration, servo_index);
    if (k_uptime_get_32() - started_ms > 500u) result = -ETIMEDOUT;
    k_mutex_unlock(&head_lock);
    k_sleep(K_MSEC(1));
    k_mutex_lock(&head_lock, K_FOREVER);
  }
  if (runtime.shutdown_requested) result = -ECANCELED;
  runtime.discovery_active = false;
  runtime.discovery_verified = result == 0;
  return result;
}

static void discovery_thread(void *first, void *second, void *third)
{
  ARG_UNUSED(first); ARG_UNUSED(second); ARG_UNUSED(third);
  while (true) {
    k_sem_take(&discovery_pending_sem, K_FOREVER);
    k_mutex_lock(&head_lock, K_FOREVER);
    const int result = discover_inventory_locked();
    if (result == 0) {
      runtime.fault = HEAD_FAULT_NONE;
      runtime.state = HEAD_HOMING_REQUIRED;
      runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
    } else {
      /* A clear-fault retry retracts the outstanding shutdown request to let
       * this sweep run. Reinstate it on failure so torque is never left
       * unverified without the supervisor trying to confirm it off. */
      head_state_fault(&runtime, HEAD_FAULT_DISCOVERY);
    }
    const uint32_t lease_token = runtime.active_lease_token;
    const uint32_t transaction_id = discovery_transaction_id;
    k_mutex_unlock(&head_lock);
    send_ack(HEAD_MSG_CLEAR_FAULT, (int8_t)result, lease_token, transaction_id);
  }
}

/* Flash erase/program latency is unbounded relative to the 2 ms control
 * period. Every save is therefore performed here, and only after shutdown has
 * been verified twice. The A/B storage layer keeps the previous generation
 * valid until the complete replacement record is committed. */
static void storage_thread(void *a, void *b, void *c)
{
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  while (true) {
    k_sem_take(&storage_pending_sem, K_FOREVER);
    while (true) {
      k_mutex_lock(&head_lock, K_FOREVER);
      /* HEAD_TORQUE_OFF_VERIFIED is only reached through a successful
       * discovery, and discovery needs a committed profile -- so a board whose
       * stored calibration is absent or no longer matches the hardware can
       * never accept the profile that would repair it. Boot leaves such a
       * board in HEAD_FAULT_CONFIGURATION with torque UNKNOWN, or in
       * HEAD_FAULT_DISCOVERY with the shutdown sweep parked in
       * SHUTDOWN_FAILED, and this wait then spins until the host times out.
       * HEAD_MSG_CLEAR_FAULT already admits the same retry when torque was
       * never commanded on; staging is torque-off and the commit path still
       * validates in full, so grant it here for the identical reason. The
       * sweep's own verification is left untouched. */
      const bool never_energized = head_state_torque_never_commanded_on(&runtime);
      const bool ready = runtime.storage_state == HEAD_STORAGE_PENDING &&
                         (runtime.torque_state == HEAD_TORQUE_OFF_VERIFIED ||
                          never_energized) &&
                         (!runtime.shutdown_requested || never_energized);
      if (ready) runtime.storage_state = HEAD_STORAGE_WRITING;
      k_mutex_unlock(&head_lock);
      if (ready) break;
      k_sleep(K_MSEC(10));
    }

    const int save_result = head_config_save(&storage_pending_calibration);
    k_mutex_lock(&head_lock, K_FOREVER);
    runtime.storage_result = save_result;
    if (save_result != 0) {
      runtime.storage_state = HEAD_STORAGE_FAILED;
      head_state_fault(&runtime, HEAD_FAULT_CONFIGURATION);
    } else if (!storage_pending_apply) {
      runtime.storage_state = HEAD_STORAGE_SUCCEEDED;
    } else {
      calibration = storage_pending_calibration;
      head_control_init(&runtime, &calibration);
      runtime.torque_state = HEAD_TORQUE_UNKNOWN;
      runtime.fault = HEAD_FAULT_NONE;
      runtime.discovery_verified = false;
      if (discover_inventory_locked() == 0) {
        runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
        runtime.discovery_verified = true;
        runtime.fault = HEAD_FAULT_NONE;
        runtime.state = HEAD_HOMING_REQUIRED;
        runtime.storage_state = HEAD_STORAGE_SUCCEEDED;
      } else {
        runtime.storage_state = HEAD_STORAGE_FAILED;
        runtime.storage_result = -ENODEV;
        head_state_fault(&runtime, HEAD_FAULT_DISCOVERY);
      }
    }
    k_mutex_unlock(&head_lock);
  }
}

/* Hardware discovery and automatic tensioning may take time or encounter a
 * damaged bus. They must never run in the host protocol context: the USB
 * CDC device is our primary recovery and diagnostics path. */
static void bringup_thread(void *a, void *b, void *c)
{
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

#if defined(HEAD_BENCH_NO_12V)
  /* This image is deliberately safe with branch power absent. Its devicetree
   * does not enable any servo UART or fan pin, and this path never calls the
   * board, Dynamixel, fan, calibration-storage, torque, or homing routines. */
  k_mutex_lock(&head_lock, K_FOREVER);
  head_config_default(&calibration);
  head_control_init(&runtime, &calibration);
  head_state_fault(&runtime, HEAD_FAULT_CONFIGURATION);
  atomic_set(&boot_complete, 1);
  k_mutex_unlock(&head_lock);
  return;
#else
  /* Initialize and release the motor buses before waiting for USB.  The first
   * write is a configuration-independent broadcast Torque Enable=0 on every
   * branch, so an actuator with an unsafe saved startup setting is not left
   * energized during host enumeration. */
  k_mutex_lock(&head_lock, K_FOREVER);
  (void)head_config_load(&calibration);
#if defined(HEAD_BENCH_J3_ID10)
  /* Volatile diagnostic profile: isolate J3 without modifying NVS. Keep the
   * same conservative values used by the prior one-servo commissioning
   * profile. This build remains motion-capable, so physical unloading and an
   * external current limit are mandatory. */
  head_config_default(&calibration);
#if defined(HEAD_BENCH_J3_ALL)
  /* Whole-branch variant: indices 10-14 are the five J3 servos.
   * HEAD_BENCH_J3_COUNT narrows it to the first N of them so the servo count
   * can be bisected: a single-servo Sync Read exchanges 88 bytes and a
   * five-servo one 440, so a failure that appears only above some N separates
   * a per-response problem from a whole-batch one. */
#ifndef HEAD_BENCH_J3_COUNT
#define HEAD_BENCH_J3_COUNT 5
#endif
  /* HEAD_BENCH_J3_START selects which J3 index the window begins at, so a
   * single servo other than 10 can be brought up on its own. With COUNT=1 this
   * homes 10, 11, 12, 13 or 14 individually -- the one-at-a-time sequence the
   * hardware review already requires, and the only path that avoids the
   * multi-member Sync Read defect. */
#ifndef HEAD_BENCH_J3_START
#define HEAD_BENCH_J3_START 10
#endif
  calibration.expected_servo_count = (uint8_t)HEAD_BENCH_J3_COUNT;
  calibration.active_servo_mask =
      (uint32_t)((1u << HEAD_BENCH_J3_COUNT) - 1u) << HEAD_BENCH_J3_START;
#else
  calibration.expected_servo_count = 1u;
  calibration.active_servo_mask = 1u << 10u;
#endif
  calibration.allow_partial_inventory = 1u;
  for (uint8_t bench_index = 10u; bench_index < 15u; ++bench_index) {
  if ((calibration.active_servo_mask & (1u << bench_index)) == 0u) continue;
  struct head_joint_config *joint = &calibration.joints[bench_index];
  joint->homing_direction = 1.0f;
  /* 2048 ticks is half a revolution on a 4096-tick encoder. The previous 100
   * ticks was under nine degrees, and at 500 ticks/s it was spent in 200 ms --
   * so the travel budget, not the timeout, ended every search, and homing only
   * succeeded when the horn already happened to rest within nine degrees of
   * its stop. The timeout must stay clear of the travel budget or it becomes
   * the new binding limit: 2048 ticks at 500 ticks/s needs ~4.1 s. */
  /* Two revolutions. The datum is tick 0 and the servo only holds a goal
   * inside its 0..4095 Position Limit, so a joint cannot stop at the equivalent
   * angle one turn up -- it has to unwind all the way down. Multi-turn Present
   * Position accumulates while the joint is pushed around by hand, so allow
   * more than the single revolution the angle itself would need. Power-cycling
   * the servo resets the turn count and brings it back inside one revolution. */
  joint->homing_max_travel_ticks = 8192;
  joint->homing_timeout_ms = 6000u;
  /* A third of the XC330-T181's 910 mA rating. The previous 100 mA was chosen
   * so a bench mistake could not hurt anything, not from what the servo needs
   * to turn: at roughly its no-load current the loop saturates just overcoming
   * the gearbox, which is what made the horn stick, break free and overshoot
   * instead of tracking the ramp. Still far too little to damage an unloaded
   * horn, and well inside the 2500 mA per-branch budget. */
  /* XC330-T181 Current Limit (38) factory default and maximum, per the ROBOTIS
   * control table. In Operating Mode 5 this is the torque budget the position
   * PID works within, so anything lower is a different controller from the one
   * the factory gains were tuned for. */
#if defined(HEAD_BENCH_J3_ALL)
  /* head_config_validate() rejects a branch whose summed operating current
   * exceeds HEAD_BRANCH_CURRENT_BUDGET_MA (2500 mA), and it sums the
   * configured Current Limit(38) of every active servo as though all of them
   * could draw it at once. Take the largest share that still fits, capped at
   * the 910 mA rating: N<=2 therefore keeps the exact value the single-servo
   * profile was commissioned with, and only N>=3 has to reduce it. That
   * matters when bisecting by servo count -- a lower Current Limit is a
   * different controller from the one the factory gains were tuned for (see
   * above), so holding it constant keeps the comparison honest.
   *
   * The budget is conservative here: homing walks one servo at a time, so the
   * real branch draw is one moving servo plus 17 mA standby each for the rest,
   * nowhere near the 3 A branch fuse. */
  joint->operating_current_ma =
      (HEAD_BRANCH_CURRENT_BUDGET_MA / HEAD_BENCH_J3_COUNT) < 910 ?
      (int16_t)(HEAD_BRANCH_CURRENT_BUDGET_MA / HEAD_BENCH_J3_COUNT) : 910;
#else
  joint->operating_current_ma = 910;
#endif
  joint->homing_current_ma = 150;
  joint->homing_following_error_ticks = 0;
  joint->homing_persistence_ms = 10u;
  joint->homing_current_limit_ma = 250;
  joint->homing_speed_ticks_per_second = 500u;
  joint->homing_backoff_ticks = 5;
  }
  head_config_finalize(&calibration);
#endif
  head_control_init(&runtime, &calibration);
  k_mutex_unlock(&head_lock);

  const int dxl_init_result = head_dxl_init();
  const int emergency_off_result = dxl_init_result == 0 ?
                                   head_dxl_emergency_torque_off() : -EIO;
  const int fan_init_result = dxl_init_result == 0 ? head_fan_init() : -EIO;

  /* Let the host finish USB configuration before discovery and conditional
   * EEPROM normalization. DTR is asserted only after enumeration; retain
   * autonomous boot after a bounded timeout. */
  const uint32_t bringup_deadline_ms = k_uptime_get_32() + 5000u;
  while (atomic_get(&usb_configured) == 0 &&
         (int32_t)(k_uptime_get_32() - bringup_deadline_ms) < 0) {
    k_sleep(K_MSEC(10));
  }

  k_mutex_lock(&head_lock, K_FOREVER);
  if (dxl_init_result != 0 || fan_init_result != 0 ||
      !head_config_validate(&calibration)) {
    head_state_fault(&runtime, HEAD_FAULT_CONFIGURATION);
  } else if (emergency_off_result != 0 ||
             head_dxl_discover(&runtime, &calibration) != 0 ||
             head_dxl_set_torque_all(&calibration, false) != 0) {
    head_state_fault(&runtime, HEAD_FAULT_DISCOVERY);
  } else {
    /* Require an explicit Home request after torque-off telemetry has seeded
     * every active servo's present position. */
    runtime.state = HEAD_HOMING_REQUIRED;
    runtime.discovery_verified = true;
    runtime.torque_state = HEAD_TORQUE_OFF_VERIFIED;
  }
  atomic_set(&boot_complete, 1);
  k_mutex_unlock(&head_lock);
#endif
}

/* The control owner is cooperative and yields only at bounded DMA waits and
 * its absolute deadline, so bulk USB CDC output cannot preempt a due tick. */
K_THREAD_DEFINE(head_control_thread, 2048, control_thread, NULL, NULL, NULL, -1, 0, -1);
K_THREAD_DEFINE(head_telemetry_thread, 2048, telemetry_thread, NULL, NULL, NULL, 2, 0, -1);
K_THREAD_DEFINE(head_bringup_thread, 2048, bringup_thread, NULL, NULL, NULL, 3, 0, -1);
K_THREAD_DEFINE(head_usb_tx_thread, 2048, usb_tx_thread, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(head_diagnostic_thread, 2048, diagnostic_thread, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(head_discovery_thread, 3072, discovery_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(head_storage_thread, 3072, storage_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
  uint8_t byte;
  struct head_frame frame;
  uint32_t last_state = 0u;
  uint32_t last_diagnostics = 0u;
  uint32_t parser_last_byte_ms = 0u;

  if (hwinfo_get_reset_cause(&boot_reset_cause) == 0) {
    (void)hwinfo_clear_reset_cause();
  } else {
    boot_reset_cause = 0u;
  }
  if (sys_csrand_get(&boot_session_id, sizeof(boot_session_id)) != 0 ||
      boot_session_id == 0u) {
    boot_session_id = sys_rand32_get() ^ k_cycle_get_32() ^ 0x48454144u;
    if (boot_session_id == 0u) boot_session_id = 1u;
  }

  /* Bring up the host-facing diagnostic/control channel before touching the
   * motor buses.  A missing branch, a bad transceiver, or an invalid stored
   * profile must never make the controller disappear from the host. */
  /* CDC0 is Zephyr's console and may carry boot/log diagnostics. CDC1 is the
   * dedicated framed control/telemetry transport used by head_ros. */
  usb = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
  if (device_is_ready(usb)) {
    if (uart_irq_callback_user_data_set(usb, usb_uart_callback, NULL) != 0 ||
        usb_enable(usb_status_callback) != 0) usb = NULL;
  } else {
    usb = NULL;
  }

  head_state_init(&runtime);
  k_thread_start(head_bringup_thread);
  while (true) {
#if !defined(HEAD_BENCH_NO_12V)
    if (atomic_get(&boot_complete) != 0 && !worker_threads_started) {
#if defined(CONFIG_WATCHDOG)
      /* Only a real start failure faults. "Not compiled in" is checked above
       * at build time and never reaches here. */
      if (hardware_watchdog_start() != 0) {
        k_mutex_lock(&head_lock, K_FOREVER);
        head_state_fault(&runtime, HEAD_FAULT_WATCHDOG);
        k_mutex_unlock(&head_lock);
      }
#endif
      k_thread_start(head_control_thread);
      k_thread_start(head_telemetry_thread);
      worker_threads_started = true;
    }
#endif
    /* Do not access CDC endpoints until the USB device stack has completed
     * descriptor configuration. Some Teensy host controllers are sensitive
     * to early line-control/poll accesses while they enumerate. */
    for (uint16_t received_bytes = 0u; received_bytes < 1024u &&
           usb != NULL && atomic_get(&usb_configured) != 0 &&
           uart_poll_in(usb, &byte) == 0; ++received_bytes) {
      const uint32_t now = k_uptime_get_32();
      if (parser.used != 0u && now - parser_last_byte_ms > 20u) parser.used = 0u;
      parser_last_byte_ms = now;
      if (head_protocol_parse_byte(&parser, byte, &frame) == 1) dispatch_frame(&frame);
    }
    if (atomic_get(&boot_complete) != 0 && k_uptime_get_32() - last_state >= 10u) {
      send_state();
      last_state = k_uptime_get_32();
    }
    if (atomic_get(&boot_complete) != 0 &&
        k_uptime_get_32() - last_diagnostics >= 100u) {
      send_diagnostics();
      last_diagnostics = k_uptime_get_32();
    }
    k_sleep(K_MSEC(1));
  }
  return 0;
}
