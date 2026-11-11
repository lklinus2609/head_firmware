#ifndef HEAD_DXL_H_
#define HEAD_DXL_H_

#include "head.h"

#define DXL_PROTOCOL_VERSION 2u
#define DXL_BROADCAST_ID 0xFEu

struct head_dxl_probe_reply {
  uint8_t servo_id;
  uint16_t model_number;
  uint8_t firmware_version;
};

/* Read-only single-ID ping capture for hardware bring-up. */
struct head_dxl_debug_ping {
  uint8_t request[10];
  uint8_t request_length;
  uint8_t received[64];
  uint8_t received_length;
  uint8_t flags;
  uint32_t first_byte_delay_us;
  uint32_t uart_error_flags;
  /* Bus/driver forensics. stale_bytes counts what the pre-request drain threw
   * away: a previous reply delivered after its own window closed shows up here,
   * which separates "reply lost in the firmware RX path" from "servo never
   * answered". The counters are branch-lifetime totals from the UART driver. */
  uint16_t stale_bytes;
  uint16_t rx_disabled_count;
  uint16_t rx_restart_failures;
  uint16_t rx_overflow_count;
};

#define HEAD_DXL_DEBUG_RX_ANY       0x01u
#define HEAD_DXL_DEBUG_HEADER_OK    0x02u
#define HEAD_DXL_DEBUG_LENGTH_OK    0x04u
#define HEAD_DXL_DEBUG_STATUS_OK    0x08u
#define HEAD_DXL_DEBUG_ID_OK        0x10u
#define HEAD_DXL_DEBUG_CRC_OK       0x20u

/* Why discovery rejected a servo. Every rejection in head_dxl_discover() is a
 * bare `continue` that collapses into a single -ENODEV, which is unusable
 * during bring-up; this records the specific check that failed. */
enum head_dxl_discovery_reason {
  HEAD_DXL_DISCOVERY_OK = 0,
  HEAD_DXL_DISCOVERY_INACTIVE,
  HEAD_DXL_DISCOVERY_NO_REPLY,
  HEAD_DXL_DISCOVERY_STATUS_ERROR,
  HEAD_DXL_DISCOVERY_MODEL,
  HEAD_DXL_DISCOVERY_FIRMWARE,
  HEAD_DXL_DISCOVERY_COMMUNICATION_READ,
  HEAD_DXL_DISCOVERY_BAUD,
  HEAD_DXL_DISCOVERY_DRIVE_MODE,
  HEAD_DXL_DISCOVERY_PROTOCOL,
  HEAD_DXL_DISCOVERY_TORQUE_ON,
  HEAD_DXL_DISCOVERY_STATUS_LEVEL,
  HEAD_DXL_DISCOVERY_SECONDARY_ID,
  HEAD_DXL_DISCOVERY_STARTUP,
  HEAD_DXL_DISCOVERY_RETURN_DELAY,
  HEAD_DXL_DISCOVERY_OPERATING_MODE,
  HEAD_DXL_DISCOVERY_VERIFY_READBACK,
  HEAD_DXL_DISCOVERY_LIMITS,
  HEAD_DXL_DISCOVERY_GAINS,
  HEAD_DXL_DISCOVERY_PROFILES,
  HEAD_DXL_DISCOVERY_WATCHDOG,
  HEAD_DXL_DISCOVERY_CURRENT_LIMIT,
};
uint8_t head_dxl_discovery_reason(uint8_t servo_index);

/* Where the telemetry path was, for the watchdog black box in main.c. Kept in
 * uncached no-init DTCM so it can survive a watchdog reset. */
enum head_dxl_trace_point {
  HEAD_DXL_TRACE_NONE = 0,
  HEAD_DXL_TRACE_CONSUME,
  HEAD_DXL_TRACE_BUILD,
  HEAD_DXL_TRACE_DRAIN,
  HEAD_DXL_TRACE_WRITE,
  HEAD_DXL_TRACE_DONE,
};
struct head_dxl_trace {
  uint32_t point;
  uint32_t branch_index;
  uint32_t drain_count;   /* read_available() calls in the current drain */
  uint32_t point_ms;
  uint32_t consume_used;      /* bytes buffered when telemetry_consume() began */
  uint32_t consume_loops;     /* parse-loop passes in the current consume */
  uint32_t bad_used_count;    /* times a buffered count above capacity was reset */
  uint32_t bad_used_value;    /* the last such out-of-range count */
};
extern struct head_dxl_trace head_dxl_trace;

/* Telemetry receive counts since boot, per branch. They separate a reply lost
 * on the wire (no byte arrived) from one damaged in transit (bytes arrived,
 * no valid packet), and say how the damaged bytes failed. */
struct head_dxl_bus_stats {
  uint32_t request_count;            /* Sync Read requests transmitted */
  uint32_t complete_count;           /* requests answered by every member */
  uint32_t silent_timeout_count;     /* timeouts with no byte received */
  uint32_t partial_timeout_count;    /* timeouts with at least one byte received */
  uint32_t received_byte_count;
  uint32_t drained_byte_count;       /* stale bytes discarded before a request */
  uint32_t resync_byte_count;        /* bytes skipped while searching for a header */
  uint32_t bad_length_count;         /* header with an impossible LEN field */
  uint32_t bad_packet_count;         /* whole packet failing CRC or structure checks */
  uint32_t unknown_id_count;         /* valid packet from an ID not on this branch */
  uint32_t servo_error_count;        /* valid packet with a nonzero error field */
  uint32_t write_failure_count;      /* Sync Read requests that failed to transmit */
  uint32_t rx_restart_failure_count;
  /* Bad packets followed by a header exactly where they end (bytes changed
   * in place) or elsewhere (bytes lost or added). */
  uint32_t bad_in_place_count;
  uint32_t bad_shifted_count;
};
void head_dxl_bus_stats(uint8_t branch_index, struct head_dxl_bus_stats *stats);
/* Telemetry replies this servo has missed since boot. */
uint32_t head_dxl_missed_reply_count(uint8_t servo_index);
/* Register address and raw status byte of the last rejected transfer. */
void head_dxl_last_transaction_error(uint16_t *address, uint8_t *status);
/* Read-only control-table access for bring-up. Tolerates a Hardware Alert so
 * that Hardware Error Status itself can be inspected on a latched servo. */
int head_dxl_debug_read(uint8_t branch_index, uint8_t servo_id, uint16_t address,
                        uint8_t length, uint8_t *data);

/* Saturating total of retried transfers: transient bus loss stays visible
 * instead of being silently absorbed by the retries. */
uint16_t head_dxl_read_retry_count(void);
/* The last Enable/homing preparation step that failed for good (after its
 * retries), as (phase << 5) | servo_index; servo 31 = no servo (phase 0, the
 * emergency torque-off). 0 = none since boot. Reported in DIAGNOSTICS. */
uint8_t head_dxl_last_preparation_failure(void);
uint32_t head_dxl_take_hardware_alert_mask(void);
int head_dxl_init(void);
/* Sequential read-only Protocol 2.0 Pings. It never writes Torque Enable,
 * Operating Mode, Goal Position, or persistent calibration data. */
int head_dxl_probe_branch(uint8_t branch_index, struct head_dxl_probe_reply *replies,
                          size_t capacity, size_t *reply_count);
int head_dxl_debug_ping(uint8_t branch_index, uint8_t servo_id,
                        struct head_dxl_debug_ping *diagnostic);
int head_dxl_discover_one(struct head_runtime *runtime,
                          const struct head_calibration *calibration, uint8_t servo_index);
int head_dxl_discover(struct head_runtime *runtime,
                      const struct head_calibration *calibration);
int head_dxl_set_torque_all(const struct head_calibration *calibration,
                            bool enabled);
/* Configuration-independent first action after the branch UARTs initialize. */
int head_dxl_emergency_torque_off(void);
/* One bounded preparation step. 0=pending, 1=complete, negative=failure.
 * The control owner excludes telemetry and storage until completion. */
int head_dxl_prepare_step(struct head_runtime *runtime,
                          const struct head_calibration *calibration);
/* Stop pending reads and drain late replies before a fault shutdown write. */
void head_dxl_abort_telemetry(struct head_runtime *runtime);
int head_dxl_write_targets(struct head_runtime *runtime,
                           const struct head_calibration *calibration,
                           uint8_t *written_branch_mask);
void head_dxl_telemetry_tick(struct head_runtime *runtime,
                            const struct head_calibration *calibration,
                            uint32_t now_ms, bool start_new);
size_t head_dxl_build_sync_write(uint8_t branch_index, const struct head_runtime *runtime,
                                 const struct head_calibration *calibration,
                                 uint8_t *output, size_t capacity);

#endif
