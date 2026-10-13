# Tether hardware and communications bench

This directory contains commissioning tools, diagnostic sketches, the laptop
bridge/client, and the tests used to establish the current bench results. It
does not contain deployment firmware. Use
`../firmware/tether_control_arduino/` for every module.

The physical result recorded below exercised one motor. Protocol version 3,
the simulator, client, and canonical firmware address two motors on master ID
0 and three motors on each remote node.

The exercised one-motor bench architecture is:

```text
laptop -- Wi-Fi/UDP --> id 0 master -- ESP-NOW --> id 1 left / id 2 right
                           |                              |
                        local CAN                     local CAN
                           |                              |
                          motor                          motor
```

The master must be configured and flashed first. Nodes 1 and 2 may be added
in either order. The laptop communicates wirelessly with the master; USB is
only needed for flashing, diagnostics, or the temporary DM-USB-CAN bridge.

## Repository map

| Path | Purpose | Board |
|---|---|---|
| `boards/esp32-s3/` | S3 wiring and temporary-bridge notes | S3 |
| `boards/xiao-esp32-c6/` | Final C6 wiring, MAC utility, CAN smoke test | C6 |
| `wireless_serial_node/` | Temporary node 1 when laptop owns DM-USB-CAN | S3 bench only |
| `pc_bridge/` | Scan/inspect/USB-CAN bridge | laptop |
| `wireless_client/` | Wireless operator console | laptop |
| `mac_probe/` | Generic station and SoftAP MAC probe | S3 and C6 |
| `can_smoke_test/` | Generic supported-TWAI smoke test | S3 and C6 |
| `wifi_ap_smoke_test/` | SoftAP-only diagnostic | S3 and C6 |
| `wifi_espnow_coexist_test/` | Incremental SoftAP/ESP-NOW diagnostic | S3 bench |
| `simulator/` | Hardware-free protocol/safety sanity model | laptop |

The obsolete standalone serial monitor proxy was removed. It replaced the
node firmware instead of remaining an ESP-NOW node and was easy to flash by
mistake. `wireless_serial_node` is the only supported temporary USB-CAN path.

## Toolchain

Use Espressif Arduino-ESP32 `4.0.0-RC1` or newer. This code requires the
supported ESP-IDF 6.1 `esp_twai.h` node API and deliberately rejects older
cores that only provide deprecated `driver/twai.h`.

As checked on 2026-10-12, Espressif's latest stable Arduino release is 3.3.12
and the latest prerelease is 4.0.0-RC1. Stable 3.3.12 is newer by date than
some older cores but is not sufficient for this code; install the 4.0
prerelease package until 4.0 becomes stable.

Select one of these Arduino IDE boards:

- Current bench: `ESP32S3 Dev Module`.
- Final hardware: `XIAO_ESP32C6`.

The same ESP-NOW and Damiao packet layouts are used on both chips. Board
selection changes the CPU target, RF-switch setup, and TWAI pins. It does not
change laptop commands or radio packet formats.

## Identities and the SoftAP MAC rule

`../firmware/tether_control_arduino/PeerConfig.h` is the identity source of truth. Tether IDs do not
come from CAN IDs or boot order:

| Tether ID | Role | Target motor mapping |
|---:|---|---|
| 0 | master | two motors: small `0x07`/`0x17`; big `0x04`/`0x14` |
| 1 | left node | three motors; types/roles incomplete and IDs not yet scanned |
| 2 | right node | three motors; types/roles incomplete and IDs not yet scanned |

Upload `mac_probe/mac_probe.ino`, or the dedicated C6 utility at
`boards/xiao-esp32-c6/xiao_c6_identity/xiao_c6_identity.ino`, and record:

- Master station MAC -> `kMasterMac`.
- Master **SoftAP MAC** -> `kMasterApMac`.
- Each node station MAC -> its row in `kNodePeers`.

The SoftAP distinction is required. The master runs AP-only, so nodes must use
the master's **SoftAP MAC** as their ESP-NOW peer. Using the master's station
MAC causes nodes to remain offline even though every board is powered and on
the same channel.

An ESP-NOW latency sketch in another repository in the Tether project works
on XIAO ESP32-C6 and has been tested between a C6 and an S3. It configures the
C6 antenna switch, tests ESP-NOW, and prints the board's station MAC. Use that
station MAC for a motor-node entry. It does not start a SoftAP or print the
master's SoftAP MAC; use this repository's `xiao_c6_identity` or `mac_probe`
to obtain `kMasterApMac`.

## Configure and flash the canonical firmware

Edit the role lines and all active motor mappings in
`../firmware/tether_control_arduino/Config.h` before each upload.

Master, always first:

```cpp
#define TETHER_ROLE TETHER_ROLE_MASTER
#define TETHER_NODE_ID 0
```

The master mapping is small `0x07`/`0x17` and big `0x04`/`0x14`. Flash it
first. Left and right mappings remain zero and `TETHER_MOTORS_CONFIGURED=0`
until all three motors on that module have been scanned.

Left node:

```cpp
#define TETHER_ROLE TETHER_ROLE_NODE
#define TETHER_NODE_ID 1
```

Right node:

```cpp
#define TETHER_ROLE TETHER_ROLE_NODE
#define TETHER_NODE_ID 2
```

Upload node firmware only after both master MACs and that node's station MAC
are filled in `PeerConfig.h`. All builds must use the same peer table and
channel. Do **not** upload `wireless_serial_node` to a C6; C6 talks directly
to its CAN transceiver through TWAI.

## Normal C6 hardware path

Every C6, including the master, needs a 3.3 V CAN transceiver:

```text
XIAO D9 / GPIO20 (TWAI TX) -> transceiver TXD
XIAO D10 / GPIO18 (TWAI RX) <- transceiver RXD
transceiver CAN_H/CAN_L/GND <-> motor CAN_H/CAN_L/GND
motor power supply              -> motor power input
```

The Waveshare transceiver is the physical-layer device. The XIAO contains the
TWAI controller but cannot drive CAN_H/CAN_L directly. Check any
standby/enable jumper or pin on the exact transceiver revision. Use 1 Mbit/s,
common ground, and 120-ohm termination at the two physical ends of each local
bus. The one-motor bench has two CAN endpoints. The master has three CAN
devices, while each remote module has four. In every case, terminate the two physical
ends of the bus; do not place termination at every device. A module does not
share its wired bus with either remote module. CAN IDs may be
reused on different modules because the buses are electrically separate.

The code compiles for XIAO ESP32-C6. The remaining C6 validation is physical:
transceiver direction/voltage, termination, RF range, and real motor replies.
Run the C6 CAN smoke test before enabling a motor if `can_rx` stays zero.

## Laptop wireless console

The master creates:

```text
SSID: tether-control
password: tether-bench
address: 192.168.4.1
UDP port: 4210
```

Join that network, then run:

```sh
cd bench
python3 wireless_client/client.py --telemetry-hz 0
```

The client sends safety heartbeats at 10 Hz even when telemetry display is
off. A one-second loss of laptop heartbeat estops all nodes. Telemetry can
interleave with terminal input, so keep `--telemetry-hz 0` while entering
commands; restart with `--telemetry-hz 2` when you want a live display.

Conservative master motion sequence (`M=0` small, `M=1` big). Remote nodes
also accept `M=2` for their third motor:

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

`pos N M q dq` selects module `N` and motor `M`, then supplies target position
in radians and positive velocity limit in rad/s. Changing only `dq` while
leaving `q` unchanged does not request more movement. `quit` sends an estop
before closing.

### Telemetry interpretation

The successful bench move changed `q` from `0.000` to `0.050` rad. Useful
flags are:

- `0038`: CAN active, motor seen, limits valid, disabled, estop clear.
- `0039`: the same state with the motor enabled.
- `003c`: estop latched.

`motor_status=0` means disabled and `motor_status=1` means enabled. These are
Damiao status values, not failures. Fault values begin at `8`. Healthy traffic
has increasing `can_rx`/`can_tx` with `fail=0` and `buserr=0`.

## Temporary S3 + laptop DM-USB-CAN path

Use this only while the S3 node has no physical CAN transceiver:

```text
laptop -- Wi-Fi --> master id 0 -- ESP-NOW --> S3 node id 1
   |                                              |
   +-- USB -> DM-USB-CAN -> motor                 +-- USB serial --+
   +----------------------------------------------------------------+
```

Flash `wireless_serial_node/wireless_serial_node.ino` to node 1. It remains an
ESP-NOW node; USB serial is only the motor transport. Close Arduino Serial
Monitor so Python can open the ESP port.

Create the laptop environment once:

```sh
cd bench/pc_bridge
python3 -m pip install --user -r requirements.txt
python3 bridge.py --list
```

With one powered motor connected, scan without enabling it:

```sh
python3 bridge.py --scan \
  --dm-port /dev/cu.usbmodem00000000050C1
```

Inspect its mapping limits:

```sh
python3 bridge.py --inspect \
  --dm-port /dev/cu.usbmodem00000000050C1 \
  --motor-id 0x04
```

Run node 1 with the small motor:

```sh
python3 bridge.py --wireless-node \
  --motor-slot 0 \
  --esp-port /dev/cu.usbmodem5C931120951 \
  --dm-port /dev/cu.usbmodem00000000050C1 \
  --motor-id 0x07 \
  --feedback-id 0x17 \
  --pmax 12.5 \
  --vmax 10 \
  --tmax 28
```

Or run node 1 with the big motor:

```sh
python3 bridge.py --wireless-node \
  --motor-slot 1 \
  --esp-port /dev/cu.usbmodem5C931120951 \
  --dm-port /dev/cu.usbmodem00000000050C1 \
  --motor-id 0x04 \
  --feedback-id 0x14 \
  --pmax 12.5 \
  --vmax 20 \
  --tmax 200
```

After the third motor is identified and inspected, use the same command with
`--motor-slot 2` and that motor's scanned IDs and limits.

Use a second terminal for `wireless_client/client.py`. Operator commands still
travel laptop -> master -> ESP-NOW node. Closing the Python bridge, losing
ESP-NOW, or losing the laptop heartbeat disables the motor.

## Diagnostics and tests

- `mac_probe`: prints station and SoftAP MAC initializers.
- `wifi_ap_smoke_test`: verifies that an AP beacon is visible.
- `wifi_espnow_coexist_test`: identifies the step at which S3 AP/ESP-NOW
  coexistence fails.
- `can_smoke_test`: transmits standard ID `0x123` and prints received frames.
- `simulator`: checks checksums, discovery order, duplicate rejection,
  availability timeout, and watchdog behavior without hardware.

Run host tests:

```sh
cd bench
python3 simulator/simulate.py
python3 -m unittest discover -s simulator -v
python3 -m unittest discover -s pc_bridge/tests -v
```

## Safety limits

Nodes boot estopped. Reset is accepted only while disabled. A 60 ms command
watchdog disables a node; a one-second laptop watchdog estops all nodes.
Enable requires recent motor feedback and limits read from that motor. Motor
faults, CAN bus-off, lost feedback, stale sequences, MAC/ID mismatch, and
unconfigured identities prevent or stop motion.

This remains bench firmware. Keep shafts unloaded for first motion and keep a
hardware motor-power cutoff within reach. Do not power multiple motors from a
supply that repeatedly enters current limit; solve the power path first.

## Primary references

- [Espressif Arduino-ESP32 releases](https://github.com/espressif/arduino-esp32/releases)
- [ESP-IDF 6.1 TWAI driver](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32c6/api-reference/peripherals/twai.html)
- [Seeed XIAO ESP32-C6 documentation](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)
