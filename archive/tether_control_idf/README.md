# Tether control — native ESP-IDF edition

> **Archived 2026-10-12.** This project is retained as a low-level TWAI and
> ESP-NOW reference. It is not maintained, is not deployment firmware, and
> predates the two-motor-per-module topology. Use
> `../../firmware/tether_control_arduino/` for current development.

Native ESP-IDF 6.1 implementation for the Seeed XIAO ESP32-C6. It uses the
supported `esp_twai.h`/`esp_twai_onchip.h` node API, not deprecated
`driver/twai.h`.

This is a buildable low-level reference, but it is **not feature-equivalent
to the Arduino deployment firmware**. Its coordinator uses USB serial and
broadcast ESP-NOW. It does not use the laptop SoftAP/UDP console,
unicast MAC table, node discovery, or laptop-link watchdog. Use this
repository's `firmware/tether_control_arduino` project for current work.

## Architecture

```text
USB serial -> coordinator C6 -- ESP-NOW --> motor-node C6
                    |                         |
             TWAI/transceiver          TWAI/transceiver
                    |                         |
                  Damiao                    Damiao
```

There are exactly three Tether modules:

| Tether ID | Role | CAN slave | Feedback/master |
|---:|---|---:|---:|
| 0 | coordinator + local motor | provision before build | provision before build |
| 1 | left/small motor | `0x07` | `0x17` |
| 2 | right/big motor | `0x04` | `0x14` |

Nodes 1 and 2 may boot in either order. The coordinator must be built and
flashed first for commissioning consistency.

## Hardware defaults

- Board/target: Seeed XIAO ESP32-C6 / `esp32c6`.
- TWAI TX: D9/GPIO20.
- TWAI RX: D10/GPIO18.
- CAN: 1 Mbit/s Classical CAN through a 3.3 V transceiver.
- ESP-NOW: channel 1, fixed 24 Mbps, power save disabled.
- XIAO onboard antenna: RF switch GPIO3 low and GPIO14 low.

The XIAO has a CAN/TWAI controller but no CAN physical layer. Every module,
including the coordinator, needs a transceiver between TX/RX and CAN_H/CAN_L.
Share ground, check the
transceiver standby/enable state, and terminate only the two ends of the local
bus.

## Safety behavior

- Nodes boot estopped.
- Reset is accepted only while enable is false.
- No fresh command for 60 ms disables the motor.
- Enable requires recent feedback and PMAX/VMAX/TMAX read from the motor.
- Motor fault status `8` or higher, CAN bus-off, or lost feedback latches
  estop.
- Commands are clamped to motor-provided limits.
- Stale, duplicate, damaged, wrong-version, and mis-sized packets are ignored.

Damiao status `0` means disabled and `1` means enabled. The telemetry label is
`motor_status`; status `1` is not a fault.

## Build and flash

```sh
cd archive/tether_control_idf
# Activate the locally installed ESP-IDF 6.1 environment first.
export TETHER_MASTER_MOTOR_SLAVE_ID=0xNN
export TETHER_MASTER_MOTOR_FEEDBACK_ID=0xNN
./scripts/build-all.sh
```

That creates `build-coordinator`, `build-node1`, and `build-node2` using the
CAN ID table above. Build one role if preferred:

```sh
./scripts/build-role.sh coordinator
./scripts/build-role.sh node1
./scripts/build-role.sh node2
```

Flash each image:

```sh
idf.py -B build-coordinator -p /dev/cu.usbmodemMASTER flash monitor
idf.py -B build-node1 -p /dev/cu.usbmodemLEFT flash monitor
idf.py -B build-node2 -p /dev/cu.usbmodemRIGHT flash monitor
```

The coordinator serial commands are:

```text
status
reset 0
pos 0 0.0 0.1
enable 0
disable 0
reset 1
pos 1 0.0 0.1
enable 1
pos 1 0.05 0.05
disable 1
estop
```

## Version status

The project is build-checked with ESP-IDF `v6.1` for target `esp32c6`. The
build emits only upstream IDF configuration/component warnings; it does not
include the removed legacy TWAI driver or suppress its deprecation warning.

Before treating this as the production coordinator, port the Arduino
firmware's SoftAP/UDP path, unicast `PeerConfig`, master SoftAP-MAC rule,
presence reporting, and laptop watchdog. Until then, it is useful for native
TWAI and ESP-NOW development but does not satisfy the wireless-laptop
requirement.

Primary references: [ESP-IDF 6.1 TWAI](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32c6/api-reference/peripherals/twai.html)
and [Seeed XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/).
