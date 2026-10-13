# Tether controls

Motor-control firmware and commissioning tools for the three-module Tether
robot.

## What to use

| Path | Purpose | Deploy it? |
|---|---|---|
| `firmware/tether_control_arduino/` | Canonical ESP32-S3/XIAO ESP32-C6 firmware | Yes |
| `bench/` | Hardware probes, smoke tests, laptop clients, USB-CAN bridge, and simulator | No; use individual tools while commissioning |
| `archive/tether_control_idf/` | Superseded native ESP-IDF experiment | No; reference only |
| `archive/legacy_one_motor_bench_firmware/` | Firmware snapshot from the original one-motor S3 test | No; historical evidence only |

The Arduino project is the canonical implementation. New protocol, safety,
and motor-control work belongs there. The bench contains focused diagnostic
sketches and host tools; it is not another firmware implementation.

## Current topology and status

The target robot has three wireless modules: master `0`, left `1`, and right
`2`. Master `0` has one Damiao motor. Left `1` and right `2` each have three
motors on their own private local CAN bus, for seven motors total. The master
motor is currently assigned:

| Motor | Slave ID | Feedback ID |
|---|---:|---:|
| master slot 0 | `0x04` | `0x14` |

The remote slave IDs are assigned by mechanical role, but their feedback IDs
and limits have not been scanned. Do not derive feedback IDs from slave IDs.

Protocol version 3 provides motor slots `0` through `2`. Master `0` exposes
slot `0`; remote nodes expose all three. Remote slot `0` is the big motor,
slot `1` is closer to the master, and slot `2` is farther from the master.
Each module runs its active
Damiao instances over one shared TWAI bus, with separate commands, feedback,
limit discovery, enable state, estop state, sequence checking, and command
watchdogs. The multi-motor path still requires physical validation before
robot deployment.

## Start here

- Deploy or edit firmware: [`firmware/tether_control_arduino/README.md`](firmware/tether_control_arduino/README.md)
- Scan motors and run hardware tests: [`bench/README.md`](bench/README.md)
- Consult the old native-IDF work only if needed: [`archive/tether_control_idf/README.md`](archive/tether_control_idf/README.md)
