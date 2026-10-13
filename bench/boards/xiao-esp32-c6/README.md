# XIAO ESP32-C6

Use Arduino IDE board `XIAO_ESP32C6` with Espressif Arduino-ESP32
`4.0.0-RC1` or newer. The canonical firmware is
[`../../../firmware/tether_control_arduino/tether_control_arduino.ino`](../../../firmware/tether_control_arduino/tether_control_arduino.ino). It selects XIAO
D9/GPIO20 for TWAI TX and D10/GPIO18 for TWAI RX and configures the onboard
antenna switch.

Protocol version 2 exposes a small and big motor slot on every module. A
successful compile confirms the target API, not physical two-motor CAN or RF
operation.

## Get MAC addresses

Upload `xiao_c6_identity/xiao_c6_identity.ino` separately to every C6 and open
Serial Monitor at 115200 baud. Record both values from the master:

- `station_mac` goes in `kMasterMac`.
- `softap_mac` goes in `kMasterApMac`.

For each motor node, put its `station_mac` in the matching `kNodePeers` row.
The master runs in AP-only mode, so nodes **must address ESP-NOW packets to the
master's SoftAP MAC**. Using the master's station MAC makes a correctly
flashed node appear offline.

An ESP-NOW latency sketch in another repository in the Tether project is
C6-compatible. It is useful for testing a C6-to-S3/C6 ESP-NOW link and for
discovering a motor node's station MAC. It does not report the master's
SoftAP MAC, so it does not replace this repository's `xiao_c6_identity` for
master provisioning.

## Two-motor C6 firmware

Edit `../../../firmware/tether_control_arduino/Config.h` and flash in this order:

1. Master: use small `0x07`/`0x17` and big `0x04`/`0x14`, then select
   `TETHER_ROLE_MASTER`, node ID `0`.
2. Left: scan both motors, fill all four IDs, set
   `TETHER_MOTORS_CONFIGURED=1`, then select node ID `1`.
3. Right: scan both motors, fill all four IDs, set
   `TETHER_MOTORS_CONFIGURED=1`, then select node ID `2`.

The master and both remote nodes each need their own 3.3 V CAN transceiver.
Wire D9/GPIO20 to transceiver TXD and D10/GPIO18 to
transceiver RXD. Then wire CAN_H, CAN_L, and common ground to that node's local
motor bus. Supply motor power separately. Terminate only the two physical ends
of each local bus. A final module has the ESP/transceiver plus two motors on
that bus; the middle device does not receive an extra termination resistor.

The C6 does not use `wireless_serial_node`: that sketch is only the temporary
S3-plus-laptop USB-CAN arrangement. On every C6 module, upload the canonical
Arduino sketch so TWAI talks directly to the local motor bus.

## What remains hardware-dependent

The two-motor code compiles for XIAO ESP32-C6 and uses the supported ESP-IDF
6.1 TWAI node API. Bench confirmation still requires the real C6 boards,
transceivers, and both motors on one bus. Check transceiver voltage, TXD/RXD
direction, standby/enable pins, common ground, 120-ohm end termination,
1 Mbit/s bitrate, and the actual motor IDs before enabling. Use
`../../can_smoke_test/can_smoke_test.ino` first if the motors never reply.
