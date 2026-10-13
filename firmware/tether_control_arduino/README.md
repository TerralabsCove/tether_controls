# Tether control — Arduino edition

This is the canonical firmware project. The diagnostic sketches and host tools
in `../../bench/` are commissioning aids, not parallel production sources.

This is the Arduino IDE deployment firmware for the three wireless Tether
modules. It supports both the ESP32-S3 bench and the final Seeed XIAO
ESP32-C6 hardware:

```text
laptop -- Wi-Fi/UDP --> id 0 master -- ESP-NOW --> id 1 left / id 2 right
                           |                              |
                        local CAN                     local CAN
                         /    \                         /    \
                      small   big                   small   big
```

It uses unicast ESP-NOW identities, laptop and node watchdogs, the supported
ESP-IDF 6.1 TWAI node API, Damiao limit discovery, CAN recovery, full-state
commands, two motors per local CAN bus, and per-motor telemetry. It includes the fixes learned on the S3
bench: delayed ESP-NOW startup after SoftAP association, transient laptop
heartbeat retry, stale-sequence rejection, motor-fault latching, and correct
interpretation of Damiao status `1` as enabled rather than an error.

## Required toolchain

Use Espressif Arduino-ESP32 `4.0.0-RC1` or newer. Select:

- `XIAO_ESP32C6` for final hardware.
- `ESP32S3 Dev Module` for the current 16N8R bench boards.

Older Arduino-ESP32 releases expose only deprecated `driver/twai.h`; this
firmware intentionally stops compilation instead of falling back to it.
As checked on 2026-10-12, 3.3.12 is Espressif's latest stable release and
4.0.0-RC1 is its latest prerelease. Use RC1 here despite the stable channel's
version display; the required supported TWAI node API comes from IDF 6.1.

## MAC provisioning

Populate `PeerConfig.h` before motion testing:

| Entry | Value |
|---|---|
| `kMasterMac` | master station MAC |
| `kMasterApMac` | master **SoftAP MAC** |
| node 1 row | left node station MAC |
| node 2 row | right node station MAC |

The distinction is required: the master operates in AP-only mode, so every
node must use the master's **SoftAP MAC** as its ESP-NOW peer. Do not copy the
station MAC into `kMasterApMac`.

Use the MAC utility in the repository's bench directory:

```text
../../bench/mac_probe/mac_probe.ino
```

It prints ready-to-copy station and SoftAP initializers on both supported
chips. The C6-specific copy is in
`../../bench/boards/xiao-esp32-c6/xiao_c6_identity/`.

## Configure and upload

Edit the role, Tether node ID, and that module's four CAN IDs in `Config.h`.
Each module has motor `0` (small) and motor `1` (big). Set
`TETHER_MOTORS_CONFIGURED` to `1` only after both motors have been physically
scanned. An unconfigured remote node still advertises over ESP-NOW for
commissioning, but motor control remains blocked. The master refuses to boot
with missing IDs.

1. Master first: `TETHER_ROLE_MASTER`, ID `0`.
2. Left: `TETHER_ROLE_NODE`, ID `1`.
3. Right: `TETHER_ROLE_NODE`, ID `2`.

| Node | Small slave/feedback | Big slave/feedback | Status |
|---:|---:|---:|---|
| 0 | `0x07` / `0x17` | `0x04` / `0x14` | configured |
| 1 | unknown | unknown | scan both motors before enabling |
| 2 | unknown | unknown | scan both motors before enabling |

Connect only one unknown motor to the DM-USB-CAN adapter at a time and run
`bench/pc_bridge/bridge.py --scan`. The firmware reads PMAX, VMAX, and TMAX
from registers 21–23 for each motor and refuses to enable that motor until its
limits and recent feedback are available.

## C6 wiring and differences from S3

No command, telemetry, or Damiao packet format changes on C6. `Config.h`
selects the hardware details at compile time:

| Board | TWAI TX | TWAI RX | Extra setup |
|---|---|---|---|
| ESP32-S3 bench | GPIO17 | GPIO18 | none |
| XIAO ESP32-C6 | D9/GPIO20 | D10/GPIO18 | onboard antenna RF switch |

All three modules, including the master, require a 3.3 V CAN transceiver.
Connect TX to transceiver TXD, RX to RXD, share ground with the motor bus, 
use 1 Mbit/s, and terminate the two physical bus ends.
A target module has three CAN devices: its ESP/transceiver and two motors.
Terminate the physical ends, not every device.
Tether IDs 0, 1, and 2 are radio/module identities and must not be confused
with Damiao CAN IDs. Because the three CAN buses are electrically separate,
motor CAN IDs may overlap between modules but must be distinct within one
module's bus.

The code is compile-verified for both targets. Real C6 verification still
requires checking the exact Waveshare board's supply voltage and
standby/enable state, termination, CAN_H/CAN_L orientation, common ground,
and actual motor replies.

## Run

Join the master's `tether-control` network (`tether-bench`), then use the
repository's bench laptop client:

```sh
cd bench
python3 wireless_client/client.py --telemetry-hz 0
```

Commands use `N` for the Tether module and `M` for the local motor:
`M=0` is small and `M=1` is big. A conservative first motion is:

```text
peers
reset 0 0
pos 0 0 0.0 0.1
enable 0 0
disable 0 0
reset 0 1
pos 0 1 0.0 0.1
enable 0 1
disable 0 1
estop
quit
```

Keep motor power on a physical cutoff. Both motors boot estopped. Each motor
requires its own recent feedback and motor-provided limits, disables after 60
ms without its own fresh command, and latches its estop on motor fault or lost
feedback. CAN bus-off affects both motors sharing that bus. The master estops
all six motor slots after one second without laptop traffic. Telemetry includes
both `id=N` and `motor=M`.

## Verification performed

- Protocol v2 two-motor firmware compiles for S3 and XIAO C6 with
  Arduino-ESP32 `4.0.0-RC1` / ESP-IDF 6.1.
- S3 bench: SoftAP, UDP client, ESP-NOW node presence, DM-USB-CAN bridge,
  Damiao scan/inspect, telemetry, reset, enable, position motion, disable, and
  estop exercised.
- Two motors on one ESP/TWAI bus and XIAO C6 hardware are not yet exercised. A
  successful compile and simulator test are not a hardware measurement.

Primary references: [Arduino-ESP32 releases](https://github.com/espressif/arduino-esp32/releases),
[ESP-IDF 6.1 TWAI](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32c6/api-reference/peripherals/twai.html),
and [Seeed XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/).
