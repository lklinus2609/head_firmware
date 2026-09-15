# `probe` resets the MCU at 57,600 baud

**Status:** open. Companion to `LINE_TEST_WATCHDOG_FIX.md`; same root cause,
different command.

## Symptom

```
headctl --port /dev/ttyACM1 probe --branch 3 --baud 57600
...
  File ".../serialposix.py", line 549, in in_waiting
    s = fcntl.ioctl(self.fd, TIOCINQ, TIOCM_zero_str)
OSError: [Errno 5] Input/output error
```

The Teensy resets and the USB CDC endpoint disappears mid-command. Identical
presentation to the line-test defect. Only affects builds with
`CONFIG_WATCHDOG` — that is, `production` and `bench-no-fan`.

## Cause

`head_dxl_probe_branch()` sweeps every ID with sequential unicast pings:

```c
for (uint16_t raw_id = 0u; raw_id < DXL_BROADCAST_ID; ++raw_id) {
```

That is 254 iterations. Each sends a 10-byte Protocol 2.0 ping (`dxl_ping()`)
and waits out a 2500 us read timeout when nothing replies. The whole sweep runs
**inline inside `dispatch_frame()`**, which holds `head_lock` from entry to
exit. The control thread needs that same mutex to feed the hardware watchdog,
configured in `hardware_watchdog_start()` as:

```c
.window = { .min = 0u, .max = 1000u },
.flags  = WDT_FLAG_RESET_SOC,
```

A one-second budget. Sweep duration is dominated by the per-ID transmit time,
which scales inversely with baud:

| Baud | TX per ID | Total per ID | 254-ID sweep | Result |
|---|---|---|---|---|
| 1,000,000 | 0.10 ms | 2.60 ms | **660 ms** | completes |
| 115,200 | 0.87 ms | 3.37 ms | **856 ms** | completes, 144 ms margin |
| 57,600 | 1.74 ms | 4.24 ms | **1076 ms** | **exceeds watchdog, resets SoC** |

This is why `probe --baud 1000000` works and `--baud 57600` never can. It is
not a bus or servo fault.

`LINE_MODE_TEST`, `UART_TX_METER_TEST`, and `UART_RX_LINE_TEST` were moved to a
sliced diagnostic worker that releases `head_lock` between slices.
`HEAD_MSG_PROBE_BRANCH` was not given the same treatment and is still inline.

## Workaround

Use `ping-debug`, which targets one ID instead of sweeping all 254. It is
bounded by a single request/response and is safe at any supported baud:

```sh
headctl --port /dev/ttyACM1 ping-debug --branch 3 --id 15 --baud 57600
```

To check whether a branch of servos is sitting at the factory baud, ping the
specific IDs you expect rather than sweeping.

## Fix

Either move the probe sweep onto the existing sliced diagnostic worker, the way
the line tests were fixed, or bound the sweep so it cannot exceed the watchdog
window — restrict it to the IDs the active profile expects, or feed the
watchdog between iterations. The sliced worker is preferable: it keeps control
supervision and USB dispatch alive, and it generalises to any future long bus
operation.

Until then, `probe` is only safe at 1 Mbps (660 ms) and 115,200 (856 ms, thin).
