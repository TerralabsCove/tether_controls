# Tether controls

Motor-control firmware and commissioning tools for the three-module Tether
robot.

## What to use

| Path | Purpose | Deploy it? |
|---|---|---|
| `firmware/tether_control_arduino/` | Canonical two-motor ESP32-S3/XIAO ESP32-C6 firmware | Yes |
| `bench/` | Hardware probes, smoke tests, laptop clients, USB-CAN bridge, and simulator | No; use individual tools while commissioning |
| `archive/tether_control_idf/` | Superseded native ESP-IDF experiment | No; reference only |
| `archive/legacy_one_motor_bench_firmware/` | Firmware snapshot from the original one-motor S3 test | No; historical evidence only |

The Arduino project is the canonical implementation. New protocol, safety,
and motor-control work belongs there. The bench contains focused diagnostic
sketches and host tools; it is not another firmware implementation.

## Current topology and status

The target robot has three wireless modules: master `0`, left `1`, and right
`2`. Each module has one small and one big Damiao motor on its own local CAN
bus. The master pair is currently assigned:

| Motor | Slave ID | Feedback ID |
|---|---:|---:|
| master small | `0x07` | `0x17` |
| master big | `0x04` | `0x14` |

The four left/right motors have not been scanned. Do not guess their IDs or
limits.

Protocol version 2 addresses motor `0` (small) and motor `1` (big)
independently. Each module runs two Damiao instances over one shared TWAI bus,
with separate commands, feedback, limit discovery, enable state, estop state,
sequence checking, and command watchdogs. The C6 and S3 builds pass. The
two-motor path still requires physical validation before robot deployment.

## Start here

- Deploy or edit firmware: [`firmware/tether_control_arduino/README.md`](firmware/tether_control_arduino/README.md)
- Scan motors and run hardware tests: [`bench/README.md`](bench/README.md)
- Consult the old native-IDF work only if needed: [`archive/tether_control_idf/README.md`](archive/tether_control_idf/README.md)
