# Sync Read fails with more than one servo on a branch

**Status:** RESOLVED 2026-09-13. Root cause was the hardware defect HW-1 in
`../../hardware/docs/KNOWN_HARDWARE_ISSUES.md` — the branch pull-up tied to
3.3 V on a 5 V logic bus. Reworking J3's R9 to `+5V` restored full five-servo
telemetry; no firmware change was needed. The artwork still carries the defect,
so the next board spin must move R1, R2, R9 and R10 to `+5V`. Return Delay Time
0 was a suspect and is exonerated — it is unchanged and multi-servo Sync Read
now works.

Keep this document for the diagnostic path and for the caution below about
reading `received=`.
**Severity:** blocking. The head cannot run without this. Twenty servos at
500 Hz control and 100 Hz telemetry is not reachable with one-at-a-time
unicast; batched Sync Read is required, not an optimisation.

## Symptom

Telemetry Sync Read returns **zero** accepted responses as soon as a branch has
more than one active servo. Measured on J3, 1 Mbps, fan-suppressed bench image:

| Active servos | `expected` | `received` | timeouts | protocol_errors | bus_errors |
|---|---|---|---|---|---|
| 1 (id 10) | 0x01 | 0x01 | — | — | — |
| 2 (ids 10-11) | 0x03 | **0x00** | 112 | 83 | 1 |
| 5 (ids 10-14) | 0x1f | **0x00** | 139 | 176 | 60 |

Servo 10's response parses cleanly when it is alone and is rejected the moment
a second servo is listed in the same request.

Downstream effect: with no telemetry the homing preparation never seeds present
positions, so torque is never enabled and Goal Position is never written. Read
back directly from servo 10 after a failed `zero-home`:

```
Torque Enable(64)  = 0      -- never enabled
Goal Position(116) = 1917   -- never written; still where the servo started
```

`zero-home` then ends at the 6000 ms `homing_timeout_ms` with `FAULT_HOMING`.
The homing fault is a symptom; the Sync Read failure is the cause.

## What is ruled out

- **Not the servos.** All five answer unicast `probe`, `ping-debug` and
  `read-register` perfectly, report `online=yes`, `hw_error=0x00`, healthy
  voltage and temperature, and every one passes discovery (`OK`).
- **Not the response timeout.** `HEAD_TELEMETRY_RESPONSE_TIMEOUT_MS` is 8 ms.
  A five-servo exchange needs ~5.1 ms (5 x 88 B at 1 Mbps, plus request and
  turnarounds), inside both that and the 10 ms poll period. Note
  `DEFAULT_DYNAMIXEL_BAUD_CHANGE.md` and the hardware review cite 4 ms; that
  figure is stale.
- **Not buffer capacity.** `DXL_TELEMETRY_BRANCH_RX_CAPACITY` is 640 B against
  440 B for five responses; the DMA buffers are 512 B each.
- **Not the request packet.** The Sync Read length field is `7 + member_count`,
  which is correct for Protocol 2.0.
- **Not the consume logic.** `telemetry_consume()` advances by `memmove` of
  `total` and decrements `used` correctly after each parsed packet.
- **Not a gradual scaling effect.** It fails completely at N=2, with
  `bus_errors` down at 1. Whatever this is, it is not progressive degradation.

## Leading hypotheses

Both remain live. `protocol_errors` exceeding `timeouts` says responses *arrive
and fail validation* rather than never arriving, which is what a collision or a
mis-framed packet looks like — not what a silent bus looks like.

1. **Inter-response turnaround (HW-1).** In a Sync Read each servo must hear
   the previous member's response end before driving its own. That gap is
   undriven and held only by the branch pull-up, which
   `../../hardware/docs/KNOWN_HARDWARE_ISSUES.md` records as tied to 3.3 V on a
   5 V-logic bus and measured at 3.1 V. Single-servo operation never exercises
   a servo-to-servo turnaround, which is exactly why N=1 works.
2. **Return Delay Time 0.** `discover_inventory_locked()` writes and verifies
   Return Delay Time(9) = 0 on every servo (`dxl.c`, reason
   `HEAD_DXL_DISCOVERY_RETURN_DELAY`). At 1 Mbps a byte is 10 us, so a member
   must detect end-of-packet and flip its transceiver RX->TX with no slack.
   Because discovery enforces the value, a non-zero delay cannot be tested from
   a U2D2 — it needs a firmware change.

These interact: HW-1 could be what makes an RDT of 0 marginal. If so, only the
pull-up fix addresses the cause, and a non-zero delay merely masks it.

## Next tests

1. **Pull-up bodge.** 4.7 kOhm from J3 pin 3 to U6 pin 5 (+5 V). Raises idle
   from ~3.1 V to ~4.5 V. Reflash the N=2 image and read `received`. Cheap,
   reversible, and required regardless.
2. **Non-zero Return Delay Time.** Write e.g. 50 (100 us) at address 9 instead
   of 0 and re-test N=2. `DEFAULT_DYNAMIXEL_BAUD_CHANGE.md` records earlier
   single-servo bench work at a 100 us delay.
3. **Capture the wire.** A logic analyser on J3 DATA during a two-member Sync
   Read settles it outright: overlapping responses prove collision, a clean
   sequence points back into the parser. The hardware review already lists
   turnaround and full-branch timing as requiring exactly this evidence and
   never having had it.

## Correction: how to read `received=`

`telemetry_received_mask` is an instantaneous value. `head_dxl_telemetry_tick()`
zeroes it when each request is issued, so sampling `headctl diagnostics` between
a request and its responses shows an empty mask on a completely healthy branch.
Several conclusions during this investigation were drawn from single samples of
it and were wrong. Judge telemetry health from the `feedback=` ages in
`headctl status` instead: single-digit milliseconds against the 10 ms poll
period means Sync Read is completing every cycle.

## Interim workaround

`HEAD_BENCH_J3_ALL` with `HEAD_BENCH_J3_COUNT=1` and `HEAD_BENCH_J3_START=`
10..14 activates one J3 servo at a time, at the full commissioned 910 mA, so
each can be zero-homed individually. This matches the one-at-a-time homing the
hardware review requires anyway.

It is a bench workaround only. **Sync Write (goal and torque broadcasts) is a
separate path — one packet out, no responses, so no contention is possible —
and is not known to be affected, but it is also not yet verified on more than
one servo.** Verifying it is coupled to this defect, because the control loop
will not run without telemetry.
