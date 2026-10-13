# ESP32-S3 16N8R bench

Use Arduino IDE board `ESP32S3 Dev Module` for the current 16N8R test boards.
The canonical
`../../../firmware/tether_control_arduino/tether_control_arduino.ino` sketch
uses GPIO17 for TWAI TX and GPIO18 for TWAI RX. `16N8R` means 16 MB flash and
8 MB octal PSRAM; this firmware does not require PSRAM.

Two S3 motor paths exist:

- `../../../firmware/tether_control_arduino`: direct CAN through a 3.3 V CAN
  transceiver for the master or either remote node. Every module owns one
  private bus with a small and big motor. This is the same architecture
  intended for C6.
- `../../wireless_serial_node`: temporary node-1 firmware used only when the
  laptop owns the DAMIAO DM-USB-CAN adapter. Run `pc_bridge/bridge.py
  --wireless-node` at the same time.

The standalone serial monitor proxy has been removed. It did not remain an
ESP-NOW node and was easy to flash by mistake.

Use `../../mac_probe/mac_probe.ino` for identity. As on C6, configure the
master station MAC as `kMasterMac`, configure its **SoftAP MAC** as
`kMasterApMac`, and configure each node by its station MAC.

Before flashing a module, provision both local motor mappings in `Config.h`
and set `TETHER_MOTORS_CONFIGURED` to `1`. The master uses the same GPIO17
TX/GPIO18 RX transceiver wiring as a direct-CAN remote node.
