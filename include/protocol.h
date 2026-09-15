#ifndef HEAD_PROTOCOL_H_
#define HEAD_PROTOCOL_H_

#include "head.h"

#define HEAD_PROTOCOL_VERSION 2u
#define HEAD_FRAME_SOF 0xA5u
#define HEAD_MAX_FRAME_PAYLOAD 512u
#define HEAD_HELLO_SCHEMA_VERSION 1u
#define HEAD_PROTOCOL_SCHEMA_HASH 0x58F30B92u
#define HEAD_FIRMWARE_VERSION_MAJOR 1u
#define HEAD_FIRMWARE_VERSION_MINOR 2u
#define HEAD_FIRMWARE_VERSION_PATCH 0u
#ifndef HEAD_FIRMWARE_BUILD_ID
#define HEAD_FIRMWARE_BUILD_ID 0x20260903u
#endif

#define HEAD_CAP_TRANSACTION_IDS       (1ull << 0u)
#define HEAD_CAP_BOUNDED_USB_TX        (1ull << 1u)
#define HEAD_CAP_TRANSMITTED_SEQUENCE  (1ull << 2u)
#define HEAD_CAP_SINGLE_SERVO_HOMING   (1ull << 3u)
#define HEAD_CAP_CONFIG_AB_STORAGE     (1ull << 4u)
#define HEAD_CAP_MCU_WATCHDOG          (1ull << 5u)
#define HEAD_CAP_PARSER_TIMEOUT        (1ull << 6u)
#define HEAD_CAP_TORQUE_READBACK       (1ull << 7u)
#define HEAD_CAP_PROPRIOCEPTION_HOLD   (1ull << 8u)
#define HEAD_CAP_ROUTING_HOLD           (1ull << 9u)

enum head_message_type {
  HEAD_MSG_HELLO = 1,
  HEAD_MSG_HELLO_REPLY,
  HEAD_MSG_ACQUIRE_LEASE,
  HEAD_MSG_RELEASE_LEASE,
  HEAD_MSG_JOINT_TARGETS,
  HEAD_MSG_HOME,
  HEAD_MSG_MAINTENANCE_CALIBRATE,
  HEAD_MSG_ENABLE,
  HEAD_MSG_DISABLE,
  HEAD_MSG_CLEAR_FAULT,
  HEAD_MSG_FAN_OVERRIDE,
  HEAD_MSG_STATE,
  HEAD_MSG_DIAGNOSTICS,
  HEAD_MSG_ACK,
  HEAD_MSG_NACK,
  HEAD_MSG_CALIBRATION_CONFIRM,
  HEAD_MSG_GET_CONFIGURATION_INFO,
  HEAD_MSG_CONFIGURATION_INFO,
  HEAD_MSG_GET_CONFIGURATION_SLOT,
  HEAD_MSG_CONFIGURATION_SLOT,
  HEAD_MSG_STAGE_CONFIGURATION_INFO,
  HEAD_MSG_STAGE_CONFIGURATION_SLOT,
  HEAD_MSG_COMMIT_CONFIGURATION,
  HEAD_MSG_PROBE_BRANCH,
  HEAD_MSG_PROBE_BRANCH_RESULT,
  /* Torque-safe lab diagnostic: hold a branch direction for a bounded time. */
  HEAD_MSG_LINE_MODE_TEST,
  HEAD_MSG_UART_TX_METER_TEST,
  HEAD_MSG_UART_RX_LINE_TEST,
  HEAD_MSG_UART_RX_LINE_TEST_RESULT,
  HEAD_MSG_DEBUG_PING,
  HEAD_MSG_DEBUG_PING_RESULT,
  HEAD_MSG_RENEW_LEASE,
  /* Hold the post-homing/backoff targets and publish feedback without
   * accepting motion commands. Disable remains the common exit command. */
  HEAD_MSG_START_PROPRIOCEPTION,
  /* Move every active servo to raw tick zero and hold it there. */
  HEAD_MSG_START_ROUTING,
  /* Torque-off commissioning aid: read any control-table register. */
  HEAD_MSG_DEBUG_READ,
  HEAD_MSG_DEBUG_READ_RESULT,
  /* Homing for a joint with no mechanical stop: drive each active servo to the
   * nearest encoder zero and take that as its reference. */
  HEAD_MSG_ZERO_HOME,
};

struct head_frame {
  uint8_t type;
  uint16_t length;
  uint8_t payload[HEAD_MAX_FRAME_PAYLOAD];
};

struct head_frame_parser {
  /* SOF + version + type + length + payload + CRC32. */
  uint8_t buffer[HEAD_MAX_FRAME_PAYLOAD + 9u];
  size_t used;
};

uint32_t head_protocol_crc32(const uint8_t *data, size_t length);
int head_protocol_encode_frame(uint8_t type, const uint8_t *payload, uint16_t length,
                      uint8_t *output, size_t output_capacity);
int head_protocol_parse_byte(struct head_frame_parser *parser, uint8_t byte,
                           struct head_frame *frame);
int head_protocol_decode_command(const struct head_frame *frame,
                                 struct head_command *command);

#endif
