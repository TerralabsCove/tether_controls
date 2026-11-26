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
| `push <deg> [speed] [cap%] [trip] [stall_ms]` | Careful move of ID 2 from where it is, then hold. Default 30 steps/s, 20 %, trip 100, stall 400 ms. Cap 5-95 %. |
| `move <id> <deg> [speed] [cap%] [trip] [stall_ms]` | The same for any servo. Default 50 steps/s, 30 %, trip 240, stall 400 ms. Holds at no more than 30 %. |
| `stat <id>` | One status line for the button page: `STAT id pos load current volt temp torque moving chip ok`. |
| `relax <id>` | Torque off, cap back to 100 %. The joint goes limp where it is. |
| `endure [sec] [n]` | ID 1 ±50° and ID 2 dock/undock over and over, rainbow on, one CSV line per cycle with the chip temperature. |
| `arm <sec>`, `disarm`, `report` | One-shot endurance run at the next power-up. Stats kept in flash. |
| `rainbow`, `rainbow <n> <sec>`, `off` | The rainbow runs by default. |
| `temp` | Chip temperature. |
| `ledpin <gpio>`, `pinhigh <gpio>` | Ring diagnostics: move the ring data pin, or hold a pin at 3.3 V for a multimeter. Both refuse the bus pin and the strapping pins. |
| `raw <id>`, `line`, `drive <0\|1>` | Bus debugging. |

Every move caps torque (register 48, RAM) and stops and holds where it is if the load reaches the trip or the
joint stops moving for the stall window (0.4 s by default). Sending `s` stops a move part-way and holds.

## Button page

`ui/server.py` holds the board's USB serial port and serves `ui/index.html` on http://127.0.0.1:8765 (this
machine only). It works with this board or the Nano.

```bash
~/.platformio/penv/bin/python ui/server.py --port /dev/cu.usbserial-2120
```

Leave out `--port` to use the first `/dev/cu.usbmodem*` or `/dev/cu.usbserial*`. Close any other serial
monitor first. The page polls `stat` and shows position, load against the cap, torque, current, temperature and
supply. It has jog buttons (`move`), go to angle, torque off, dock and undock, rainbow, endurance, scan and a raw
command box. STOP sends `s`, then `relax` once the board is idle.

The jog buttons and mid-move STOP had not been tried on hardware when this was written (2026-11-25).

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

The ring stayed dark on D3 with the second loom on 2026-11-24, although it had 5 V. The ring's data line was
being checked with `pinhigh 7`. A 5 V ring may not see the C5's 3.3 V data as high (WS2812B wants about 0.7 ×
5 V = 3.5 V). It did light from GPIO0 on the first loom.
