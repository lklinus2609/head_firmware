#include <string.h>

#include "protocol.h"

static uint32_t get_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
         ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
}

static uint16_t get_u16_le(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8u));
}

static float get_f32_le(const uint8_t *data)
{
  const uint32_t bits = get_u32_le(data);
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

uint32_t head_protocol_crc32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFFu;

  for (size_t byte_index = 0; byte_index < length; ++byte_index) {
    crc ^= data[byte_index];
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      crc = (crc >> 1u) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
  }
  return ~crc;
}

int head_protocol_encode_frame(uint8_t type, const uint8_t *payload, uint16_t length,
                      uint8_t *output, size_t output_capacity)
{
  const size_t total = (size_t)length + 9u;
  uint32_t crc;

  if (length > HEAD_MAX_FRAME_PAYLOAD || output_capacity < total ||
      (length != 0u && payload == NULL) || output == NULL) {
    return -1;
  }
  output[0] = HEAD_FRAME_SOF;
  output[1] = HEAD_PROTOCOL_VERSION;
  output[2] = type;
  output[3] = (uint8_t)(length & 0xFFu);
  output[4] = (uint8_t)(length >> 8u);
  if (length != 0u) {
    memcpy(&output[5], payload, length);
  }
  crc = head_protocol_crc32(&output[1], (size_t)length + 4u);
  output[5u + length] = (uint8_t)crc;
  output[6u + length] = (uint8_t)(crc >> 8u);
  output[7u + length] = (uint8_t)(crc >> 16u);
  output[8u + length] = (uint8_t)(crc >> 24u);
  return (int)total;
}

static int parser_recover(struct head_frame_parser *parser,
                          struct head_frame *frame)
{
  size_t incomplete_start = parser->used;
  /* Prefer a complete valid suffix. This recovers a good frame that arrived
   * after a truncated/corrupt one without waiting for another host write. */
  for (size_t start = 1u; start < parser->used; ++start) {
    if (parser->buffer[start] != HEAD_FRAME_SOF) continue;
    const size_t available = parser->used - start;
    if (available < 5u) {
      incomplete_start = start;
      continue;
    }
    if (parser->buffer[start + 1u] != HEAD_PROTOCOL_VERSION) continue;
    const uint16_t length = get_u16_le(&parser->buffer[start + 3u]);
    if (length > HEAD_MAX_FRAME_PAYLOAD) continue;
    const size_t expected = (size_t)length + 9u;
    if (available < expected) {
      incomplete_start = start;
      continue;
    }
    const uint32_t expected_crc = get_u32_le(
        &parser->buffer[start + 5u + length]);
    const uint32_t actual_crc = head_protocol_crc32(
        &parser->buffer[start + 1u], (size_t)length + 4u);
    if (expected_crc != actual_crc) continue;
    frame->type = parser->buffer[start + 2u];
    frame->length = length;
    if (length != 0u) {
      memcpy(frame->payload, &parser->buffer[start + 5u], length);
    }
    parser->used = 0u;
    return 1;
  }
  if (incomplete_start < parser->used) {
    const size_t retained = parser->used - incomplete_start;
    memmove(parser->buffer, &parser->buffer[incomplete_start], retained);
    parser->used = retained;
  } else {
    parser->used = 0u;
  }
  return -1;
}

int head_protocol_parse_byte(struct head_frame_parser *parser, uint8_t byte,
                           struct head_frame *frame)
{
  size_t expected;
  uint16_t length;
  uint32_t expected_crc;
  uint32_t actual_crc;

  if (parser->used == 0u && byte != HEAD_FRAME_SOF) {
    return 0;
  }
  if (parser->used >= sizeof(parser->buffer)) {
    return parser_recover(parser, frame);
  }
  parser->buffer[parser->used++] = byte;
  if (parser->used < 5u) {
    return 0;
  }
  if (parser->buffer[1] != HEAD_PROTOCOL_VERSION) {
    return parser_recover(parser, frame);
  }
  length = get_u16_le(&parser->buffer[3]);
  if (length > HEAD_MAX_FRAME_PAYLOAD) {
    return parser_recover(parser, frame);
  }
  expected = (size_t)length + 9u;
  if (parser->used < expected) {
    return 0;
  }
  if (parser->used != expected) {
    return parser_recover(parser, frame);
  }
  expected_crc = get_u32_le(&parser->buffer[5u + length]);
  actual_crc = head_protocol_crc32(&parser->buffer[1], (size_t)length + 4u);
  if (expected_crc != actual_crc) {
    return parser_recover(parser, frame);
  }
  frame->type = parser->buffer[2];
  frame->length = length;
  if (length != 0u) {
    memcpy(frame->payload, &parser->buffer[5], length);
  }
  parser->used = 0u;
  return 1;
}

int head_protocol_decode_command(const struct head_frame *frame,
                                 struct head_command *command)
{
  const size_t expected = 16u + sizeof(command->position_normalized) +
                          sizeof(command->velocity_normalized);

  if (frame->type != HEAD_MSG_JOINT_TARGETS || frame->length != expected) {
    return -1;
  }
  command->lease_token = get_u32_le(&frame->payload[0]);
  command->sequence = get_u32_le(&frame->payload[4]);
  command->active_servo_mask = get_u32_le(&frame->payload[8]);
  command->mode = frame->payload[12];
  if (frame->payload[13] != 0u || frame->payload[14] != 0u || frame->payload[15] != 0u) {
    return -1;
  }
  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    command->position_normalized[index] =
        get_f32_le(&frame->payload[16u + index * sizeof(float)]);
    command->velocity_normalized[index] = get_f32_le(
        &frame->payload[16u + sizeof(command->position_normalized) +
                        index * sizeof(float)]);
  }
  return 0;
}
