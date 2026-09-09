#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "protocol.h"

static void test_v2_command_round_trip(void)
{
  uint8_t payload[16u + 2u * HEAD_SERVO_COUNT * sizeof(float)] = {0};
  uint8_t encoded[HEAD_MAX_FRAME_PAYLOAD + 9u];
  struct head_frame_parser parser = {0};
  struct head_frame frame;
  struct head_command command;
  const uint32_t token = 0x12345679u;
  const uint32_t sequence = 42u;
  const uint32_t mask = 0x00000FFFu;
  float position[HEAD_SERVO_COUNT];
  float velocity[HEAD_SERVO_COUNT] = {0};

  for (uint8_t index = 0u; index < HEAD_SERVO_COUNT; ++index) {
    position[index] = (float)index / (float)(HEAD_SERVO_COUNT - 1u);
  }
  memcpy(&payload[0], &token, sizeof(token));
  memcpy(&payload[4], &sequence, sizeof(sequence));
  memcpy(&payload[8], &mask, sizeof(mask));
  payload[12] = HEAD_COMMAND_POSITION;
  memcpy(&payload[16], position, sizeof(position));
  memcpy(&payload[16u + sizeof(position)], velocity, sizeof(velocity));

  const int length = head_protocol_encode_frame(HEAD_MSG_JOINT_TARGETS, payload,
                                       sizeof(payload), encoded, sizeof(encoded));
  assert(length == (int)(sizeof(payload) + 9u));
  int parsed = 0;
  for (int index = 0; index < length; ++index) {
    parsed = head_protocol_parse_byte(&parser, encoded[index], &frame);
  }
  assert(parsed == 1);
  assert(head_protocol_decode_command(&frame, &command) == 0);
  assert(command.lease_token == token);
  assert(command.sequence == sequence);
  assert(command.active_servo_mask == mask);
  assert(command.mode == HEAD_COMMAND_POSITION);
  assert(memcmp(command.position_normalized, position, sizeof(position)) == 0);
  assert(memcmp(command.velocity_normalized, velocity, sizeof(velocity)) == 0);
}

static void test_crc_and_reserved_bytes_are_rejected(void)
{
  uint8_t payload[16u + 2u * HEAD_SERVO_COUNT * sizeof(float)] = {0};
  uint8_t encoded[HEAD_MAX_FRAME_PAYLOAD + 9u];
  struct head_frame_parser parser = {0};
  struct head_frame frame;
  struct head_command command;
  int length = head_protocol_encode_frame(HEAD_MSG_JOINT_TARGETS, payload,
                                 sizeof(payload), encoded, sizeof(encoded));
  encoded[20] ^= 0x01u;
  int parsed = 0;
  for (int index = 0; index < length; ++index) {
    parsed = head_protocol_parse_byte(&parser, encoded[index], &frame);
  }
  assert(parsed == -1);

  memset(&frame, 0, sizeof(frame));
  frame.type = HEAD_MSG_JOINT_TARGETS;
  frame.length = sizeof(payload);
  frame.payload[13] = 1u;
  assert(head_protocol_decode_command(&frame, &command) == -1);
}

static void test_maximum_payload_round_trip(void)
{
  uint8_t payload[HEAD_MAX_FRAME_PAYLOAD];
  uint8_t encoded[HEAD_MAX_FRAME_PAYLOAD + 9u];
  struct head_frame_parser parser = {0};
  struct head_frame frame;

  for (size_t index = 0u; index < sizeof(payload); ++index) {
    payload[index] = (uint8_t)index;
  }
  const int length = head_protocol_encode_frame(HEAD_MSG_CONFIGURATION_SLOT, payload,
                                       sizeof(payload), encoded, sizeof(encoded));
  assert(length == (int)sizeof(encoded));
  int parsed = 0;
  for (int index = 0; index < length; ++index) {
    parsed = head_protocol_parse_byte(&parser, encoded[index], &frame);
  }
  assert(parsed == 1);
  assert(frame.length == sizeof(payload));
  assert(memcmp(frame.payload, payload, sizeof(payload)) == 0);
}

static void test_invalid_encoder_arguments_are_rejected(void)
{
  uint8_t encoded[HEAD_MAX_FRAME_PAYLOAD + 9u];
  assert(head_protocol_encode_frame(HEAD_MSG_HELLO, NULL, 1u, encoded,
                           sizeof(encoded)) == -1);
  assert(head_protocol_encode_frame(HEAD_MSG_HELLO, NULL, 0u, NULL, 0u) == -1);
}

static void test_proprioception_request_round_trip(void)
{
  const uint8_t payload[8] = { 7u, 0u, 0u, 0u, 42u, 0u, 0u, 0u };
  uint8_t encoded[32];
  struct head_frame_parser parser = {0};
  struct head_frame frame;
  const int length = head_protocol_encode_frame(HEAD_MSG_START_PROPRIOCEPTION, payload,
                                       sizeof(payload), encoded, sizeof(encoded));
  assert(length == 17);
  int parsed = 0;
  for (int index = 0; index < length; ++index) {
    parsed = head_protocol_parse_byte(&parser, encoded[index], &frame);
  }
  assert(parsed == 1);
  assert(frame.type == HEAD_MSG_START_PROPRIOCEPTION);
  assert(frame.length == sizeof(payload));
  assert(memcmp(frame.payload, payload, sizeof(payload)) == 0);
}

static void test_routing_request_round_trip(void)
{
  assert(HEAD_MSG_START_ROUTING == 34);
  const uint8_t payload[8] = { 7u, 0u, 0u, 0u, 43u, 0u, 0u, 0u };
  uint8_t encoded[32];
  struct head_frame_parser parser = {0};
  struct head_frame frame;
  const int length = head_protocol_encode_frame(HEAD_MSG_START_ROUTING, payload,
                                       sizeof(payload), encoded, sizeof(encoded));
  assert(length == 17);
  int parsed = 0;
  for (int index = 0; index < length; ++index) {
    parsed = head_protocol_parse_byte(&parser, encoded[index], &frame);
  }
  assert(parsed == 1);
  assert(frame.type == HEAD_MSG_START_ROUTING);
  assert(frame.length == sizeof(payload));
  assert(memcmp(frame.payload, payload, sizeof(payload)) == 0);
}

static void test_parser_recovers_embedded_valid_suffix(void)
{
  uint8_t valid[32];
  uint8_t stream[64];
  const uint8_t transaction[4] = { 1u, 0u, 0u, 0u };
  struct head_frame_parser parser = {0};
  struct head_frame frame;
  const int valid_length = head_protocol_encode_frame(HEAD_MSG_HELLO, transaction,
                                              sizeof(transaction), valid,
                                              sizeof(valid));
  assert(valid_length == 13);
  size_t byte_offset = 0u;
  stream[byte_offset++] = HEAD_FRAME_SOF;
  stream[byte_offset++] = HEAD_PROTOCOL_VERSION;
  stream[byte_offset++] = HEAD_MSG_STATE;
  stream[byte_offset++] = 13u;
  stream[byte_offset++] = 0u;
  memcpy(&stream[byte_offset], valid, (size_t)valid_length);
  byte_offset += (size_t)valid_length;
  memset(&stream[byte_offset], 0x55, 4u);
  byte_offset += 4u;

  int parsed = 0;
  for (size_t index = 0u; index < byte_offset; ++index) {
    parsed = head_protocol_parse_byte(&parser, stream[index], &frame);
  }
  assert(parsed == 1);
  assert(frame.type == HEAD_MSG_HELLO);
  assert(frame.length == sizeof(transaction));
  assert(memcmp(frame.payload, transaction, sizeof(transaction)) == 0);
}

int main(void)
{
  test_v2_command_round_trip();
  test_crc_and_reserved_bytes_are_rejected();
  test_maximum_payload_round_trip();
  test_invalid_encoder_arguments_are_rejected();
  test_proprioception_request_round_trip();
  test_routing_request_round_trip();
  test_parser_recovers_embedded_valid_suffix();
  return 0;
}
