# STS3215 + LED ring bench test (XIAO ESP32-C5)

A bench sketch, not deployment firmware. It drives FEETECH STS3215 12 V serial bus servos straight from one
XIAO ESP32-C5 GPIO, with no adapter board, and runs a WS2812B ring. It has the same commands as the Nano version
in `../../arduino-nano/sts3215_led_test/`.

Results are summarized in the tether_context notes `notes/2026-11-24-john-sts3215-c5-bench.md` and
`notes/2026-11-24-john-nano-c5-docking-endurance.md`.

## Boards

| Label | Base / station MAC  | SoftAP MAC          | Location                              |
|-------|---------------------|---------------------|---------------------------------------|
| C5 #1 | `38:44:BE:1F:F9:DC` | `38:44:BE:1F:F9:DD` | End effector, the one closest to 11   |
| C5 #2 | `10:BD:A3:CD:D1:B8` | `10:BD:A3:CD:D1:B9` | Not yet assigned                      |

Read with `esptool --port <port> read-mac` on 2026-11-24. The SoftAP MAC is the base MAC + 1, the ESP32 default.
Station initializers: C5 #1 `{0x38, 0x44, 0xBE, 0x1F, 0xF9, 0xDC}`, C5 #2 `{0x10, 0xBD, 0xA3, 0xCD, 0xD1, 0xB8}`.

## Wiring

| Signal | XIAO pin | GPIO |
|---|---|---|
| Servo data (one wire, servos daisy-chained) | D2 | 25 |
| LED ring DIN | D3 | 7 |
| GND (servo pack, buck, ring) | GND | |

Neither GPIO25 nor GPIO7 is a boot-strapping pin. The C5's strapping pins are GPIO2, 3, 26, 27 (on-board LED)
and 28.

The servos run off the 3S pack. The ring runs off the 5 V buck.

## Power: USB or buck, never both

A C5 running on the buck's 5 V while it was also plugged into the laptop got hot and died (2026-11-24).

- **Bench (USB):** the buck's + lead is off the C5's 5V pad. The buck may still power the ring.
- **Untethered (buck):** unplug USB and the pack. Wire grounds first, then the buck's 5 V to the 5V pad, the
  ring and the servo data. Plug the pack in last, so the buck's output ramps up into a wired load instead of a
  live 5 V line being plugged into the C5.
- **Back to USB:** unplug the pack first, then take the buck's 5 V off the C5, then plug in USB.

Check before connecting: the buck reads 5.0-5.2 V, and the servo plug order is pin 1 GND, pin 2 12 V, pin 3
signal. A swapped crimp puts 12 V on D2.

## Build and flash

```bash
pio run -e xiao_c5 -t upload
```

Uses the pioarduino platform (Arduino-ESP32 3.3.x) and Adafruit NeoPixel. The `xiao_s3` env builds with the same
D2/D3 pin names but is untested.

## Commands

Over USB serial at 115200. Use `bench_cli.py` or `endure_log.py`:

```bash
~/.platformio/penv/bin/python bench_cli.py "temp" "scan" "dock" "undock"
~/.platformio/penv/bin/python endure_log.py --log run.log --args "300 16"
```

| Command | What it does |
|---|---|
| `scan [maxid]` | Ping IDs 0..maxid. Read only. |
| `swing <id> <deg> [tq%]`, `go <id> <deg> [tq%]`, `seq <id> <p1> [p2]` | Guarded test moves. |
| `dock [speed] [trip]` | ID 2 out to 1660 (145.9°), then hold with torque on at a 30 % cap. Default 50 steps/s, trip 160. |
| `undock [speed] [trip]` | ID 2 back to 1080 (94.9°), then torque off. |
| `push <deg> [speed] [cap%] [trip]` | Careful move of ID 2 from where it is, then hold. Default 30 steps/s, 20 %, trip 100. |
| `endure [sec] [n]` | ID 1 ±50° and ID 2 dock/undock over and over, rainbow on, one CSV line per cycle with the chip temperature. |
| `arm <sec>`, `disarm`, `report` | One-shot endurance run at the next power-up. Stats kept in flash. |
| `rainbow`, `rainbow <n> <sec>`, `off` | The rainbow runs by default. |
| `temp` | Chip temperature. |
| `ledpin <gpio>`, `pinhigh <gpio>` | Ring diagnostics: move the ring data pin, or hold a pin at 3.3 V for a multimeter. Both refuse the bus pin and the strapping pins. |
| `raw <id>`, `line`, `drive <0\|1>` | Bus debugging. |
| `ofs <id> [steps]` | Read, or write to EEPROM, the position offset (reg 31, sign-magnitude; which way the sign shifts the reading is not yet checked). Torque must be off. |
| `setid <id> <new>` | Change a servo's bus id (EEPROM). Torque must be off; refused if `<new>` already answers. Only one servo with `<id>` on the bus. |

Every move caps torque (register 48, RAM) and stops and holds where it is if the load reaches the trip or the
joint stops moving for 0.4 s.

## Buffer board build and debug commands over the radio

`pio run -e xiao_c5_ee2_buf -t upload` builds EE2 for the servo bus buffer board (74LS125, drawing TC-EE2-BUF-001):
TX D7 (GPIO12), RX D6 (GPIO11), DIR D8 (GPIO8, low = send) instead of one wire on D2. The ESP hears its own bytes as
an echo; the build checks the echo, skips it, and retries a read or write up to 3 times. `bustest` checks the buffer
loop with no servo: RX must follow TX while DIR is low.

Board changes that this build was run with on 2026-11-26 (tether_context note
`notes/2026-11-26-steve-ee2-servo-buffer-debug.md`):

- R6 (the DATA pull-up to +5V, 1.1 kΩ on this board) goes to U1 pin 3, the chip side of R4, not to DATA. On the DATA
  side the pull-up current crossed R4 and a low on DATA measured 1.04 V; the servo's spec sheet wants under 0.45 V.
  After the move it measured 0.13 V.
- The servo's GND and the pack's − must be joined to the XIAO's GND. On EE2's board that link comes through the
  buck, so with the XIAO on a laptop's USB and the buck off, the servo does not answer.

Every `EE_ID` build also takes debug commands over the radio, for any servo id and outside the `EE_LIM` goal limits
(tether_ros `tools/ee_debug.py`). A debug command is a T_EE_CMD frame with flag 0x80; the reply is a
T_EE_STATE-shaped frame with `ee | 0x80`, so the tethersmall gateway forwards both unchanged. Guards: arm first
(lapses after 2 min idle), torque cap 5-60 %, speed 10-1000 steps/s, the same load and stall stop as every bench
move, registers below 40 (EEPROM) refused, torque off 10 s after the last debug command, and no debug move while ROS
commands are active.

Tested: EE2's servo 2 swung 30° each way through Board 10 with `ee_debug.py`, every leg within 0.7° of its target.
Not tested: the ROS goal path (T_EE_CMD follow) on this build, a servo at ID 1, and the reply error rate.

`../../xiao-esp32-s3/sts3215_buf_test/` is a smaller sketch for a XIAO ESP32-S3 on the same board (arduino-cli):
`bustest`, `low` (hold DATA low for a multimeter), `bauds`, `scan`, `stat`, `swing`. Its loop test and echo passed on
the board; it has not had a reply from a servo, and its TX pin was changed from D9 to D7 afterwards without a re-run.

## How the one-wire bus is driven

TX and RX share the pad. The pad is open drain with pull-up while listening. While sending it is
push-pull. It switches back to open drain as soon as the UART reports the last stop bit is out.

With open drain only, the servo never answered at any ID or baud rate. Our own bytes echoed back clean.
Pushing the line high fixed it at once. The likely cause is a weak pull-up in the servo giving slow
rising edges at 1 Mbit/s. That is assumed, not measured with a scope.

The servo's data line idled at about 3.07 V against the servo's own GND. FEETECH's spec sheet for the
12 V model (ST-3215-C018, edition A/0) gives signal high 2-5 V and low 0-0.45 V, so 3.3 V and 5 V boards are
both in spec. Its reference circuit pulls DATA up with 10 kΩ to the MCU supply and drives it through
74LVC1G126 / 74LVC1G125 buffers, which fits the slow-edge explanation above.

The same sheet allows 38400 bit/s to 1 Mbit/s (default 1 Mbit/s). It rates the servo at 12 V (4-14 V),
2.7 A at stall. The servo cuts output by itself after 2 s over 80 % of stall load or over 2 A; the next
position command clears that. The guard in this sketch trips well before.

## Open

EE1 (C5 #1) bus, 2026-11-24:
- **Servo IDs:** the second servo came with the factory ID 1 and was set to 2 with `setid 1 2`. That was checked
  with a clean ping at ID 2 while ID 1 went silent. It has not yet been checked after a power cycle.
- **Weak data line:** replies were mostly corrupt, roughly 1 ping in 6 clean, every error a 1 read as 0 and most at
  the start of the reply. Fixing the ground did not help. With `drive 0` (open drain) even our own echo was corrupt.
  The line's high level is weak or slow on this loom. Try an external 1-2.2 kΩ pull-up from D2 to 3V3.
- **Servo 1 sits at 4092 steps**, at the 4095 -> 0 wrap. That is outside the EE firmware's 2250-3450 limit for ID 1,
  so a ROS goal near it gets clamped about 56° away. Keep EE1 out of ROS commands until `ofs 1 <n>` makes it read
  about 2848 at rest, like EE2's. One clean read showed the offset already at 85.

The ring stayed dark on D3 with the second loom on 2026-11-24, although it had 5 V. The ring's data line was
being checked with `pinhigh 7`. A 5 V ring may not see the C5's 3.3 V data as high (WS2812B wants about 0.7 ×
5 V = 3.5 V). It did light from GPIO0 on the first loom.
