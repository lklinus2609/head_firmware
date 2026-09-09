# Default Dynamixel baud change

## Reason

The commissioned servos are configured for Dynamixel Protocol 2.0 at 1 Mbps,
but production firmware initialized every branch at 3 Mbps. Diagnostic commands
temporarily switched a selected branch to 1 Mbps and restored it to 3 Mbps after
each request, adding unnecessary UART/DMA reconfiguration during bring-up.

## Before

- J1 through J4 initialized at 3 Mbps in devicetree and `board.c`.
- `probe` and `ping-debug` restored the selected branch to 3 Mbps.
- Discovery required the servo Baud Rate register to contain the 3 Mbps value.
- The documented production Dynamixel contract specified 3 Mbps.

## After

- J1 through J4 initialize at 1 Mbps.
- A shared `HEAD_BRANCH_DEFAULT_BAUD` constant defines the runtime default.
- `probe` and `ping-debug` restore the selected branch to 1 Mbps.
- Selecting the rate already in use is a no-op, avoiding needless UART and RX
  DMA restarts during repeated 1 Mbps diagnostics.
- Discovery requires the Dynamixel Baud Rate register value for 1 Mbps (`3`).
- Production documentation now specifies Protocol 2.0 at 1 Mbps.

Diagnostic probing at 57,600, 115,200, 2 Mbps, 3 Mbps, or 4 Mbps remains
available when an explicit `--baud` value is supplied.

## Verification

1. Build and flash the production image.
2. Keep the known-good servo on J2 or J3, configured as ID 3, Protocol 2.0,
   1 Mbps, with torque off.
3. Run repeated `ping-debug --baud 1000000` requests.
4. Confirm CRC-valid replies and compare the success rate with the earlier
   intermittent tests.
5. Treat J1 separately: its DATA net was measured near ground while J2 through
   J4 idled at 3.3 V, so this baud change does not repair J1's hardware fault.

## Bench result

Using the same ID 3 servo on J3 at 1 Mbps with a 100 us Return Delay Time:

- Before this change: 6 of 31 pings returned a complete CRC-valid status packet
  (19.4%).
- After this change: 23 of 31 pings returned a complete CRC-valid status packet
  (74.2%).

Eliminating the 3 Mbps to 1 Mbps transition therefore removed a major source of
failure. The remaining 8 misses produced no bytes and no UART error flags, so
the link is still not acceptable for production. Further diagnosis must
separate a missing servo response on the shared DATA wire from a response that
reaches the Teensy RX pin but is not delivered by the UART driver.
