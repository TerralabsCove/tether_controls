# STS3215 + LED ring bench test (Arduino Nano)

A bench sketch, not deployment firmware. It drives FEETECH STS3215 12 V serial bus servos from one Nano pin
and runs a WS2812B ring. It was used on 2026-11-24 to test the servos, the docking end effector and two
wiring looms on a cheap board before going back to the XIAO ESP32-C5. The C5 version with the same commands
is in `../../xiao-esp32-c5/sts3215_led_test/`.

Results are summarized in the tether_context note `notes/2026-11-24-john-nano-c5-docking-endurance.md`.

## Wiring

| Signal | Nano pin |
|---|---|
| Servo data (one wire, servos daisy-chained) | D2 |
| LED ring DIN | D3 |
| GND (servo pack, buck, ring) | GND |

The servos run off the 3S pack. The ring runs off the 5 V buck. The Nano runs off USB, or off the buck on its
5V pin (not VIN) for untethered runs. Wire everything with the pack unplugged, then plug the pack in last.

### Why D2 and not RX/TX

The Nano's only UART (D0/D1) is shared with the USB chip. With the servo data on D1 jumpered to D0, the Nano
could talk to the servos, but once the servos were powered the PC could no longer send commands to the Nano:
its USB chip drives D0 through a 1 kΩ resistor, which could not pull the servo line low. So the bus runs on D2,
bit-banged at 1 Mbit/s in hand-timed assembly (`src/sbus.S`): 16 cycles per bit at 16 MHz, interrupts off for
each packet. D2 is push-pull while sending (5 V high, inside the servo's 2-5 V spec) and an input with the
internal pull-up while listening. It worked on the first scan.

## Build and flash

```bash
pio run -t upload
```

Board `nanoatmega328new` (Optiboot, 115200 upload). Uses Adafruit NeoPixel.

## Commands

Over USB serial at 115200. Use `bench_cli.py`, which sends a wake byte first so a command is not lost while
the ring is being updated:

```bash
~/.platformio/penv/bin/python bench_cli.py "scan" "dock" "undock"
~/.platformio/penv/bin/python endure_log.py --log run.log --args "300 16"
```

| Command | What it does |
|---|---|
| `scan [maxid]` | Ping IDs 0..maxid. Read only. |
| `swing <id> <deg> [tq%]` | +deg, back, -deg, back. |
| `go <id> <deg> [tq%]` | One move by deg, then back. |
| `seq <id> <p1> [p2]` | Guarded moves to absolute positions. |
| `dock [speed] [trip]` | ID 2 out to 1660 (145.9°), then hold with torque on at a 30 % cap. Default 50 steps/s, trip 160. |
| `undock [speed] [trip]` | ID 2 back to 1080 (94.9°), then torque off. Same defaults. |
| `push <deg> [speed] [cap%] [trip] [stall_ms]` | Careful move of ID 2 from where it is, then hold. Default 30 steps/s, 20 %, trip 100, stall 400 ms. Cap 5-95 %. |
| `move <id> <deg> [speed] [cap%] [trip] [stall_ms]` | The same for any servo. Default 50 steps/s, 30 %, trip 240, stall 400 ms. Holds at no more than 30 %. |
| `stat <id>` | One status line for the button page: `STAT id pos load current volt temp torque moving chip ok`. |
| `relax <id>` | Torque off, cap back to 100 %. The joint goes limp where it is. |
| `endure [sec] [n]` | ID 1 ±50° and ID 2 dock/undock over and over, rainbow on, one CSV line per cycle. |
| `arm <sec>`, `disarm`, `report` | One-shot endurance run at the next power-up (for buck power, no PC). Stats kept in EEPROM. Ring ends green (clean) or red (fault). |
| `rainbow`, `rainbow <n> <sec>`, `off` | The rainbow runs by default. |
| `raw <id>`, `line` | Bus debugging. |

For buttons instead of typed commands, use the button page in
`../../xiao-esp32-c5/sts3215_led_test/ui/` with `--port /dev/cu.usbserial-*`. Its board chip reads "–" here,
because the Nano has no temperature sensor.

Speed goes up to 3072 steps/s, the 12 V spec sheet's no-load 0.222 s/60° (it was capped at 300 before
2026-11-25). Full speed has not been tried on the joints yet.

Every move caps torque (register 48, RAM) and stops and holds where it is if the load reaches the trip or the
joint stops moving for the stall window (0.4 s by default). Sending `s` stops a move part-way and holds. Endurance stops by itself at a servo temperature of 60 °C, a servo supply under
10.5 V, a servo it cannot read, or 3 blocked moves.

## Positions

These are for the arm on the bench on 2026-11-24, ID 2 driving the docking end effector.

- **Dock:** 1660 (145.9°). Slow docks stopped at 145.2°. At the 20 % cap the end effector stalls at 147.5°.
- **Retract end:** 1080 (94.9°). Every undock after the first dock stopped near 91° (load trip). Before the first
  dock, ID 2 moved freely down to 11°.
- **Dock settings:** 50 steps/s with a load trip of 160. The first dock at 150 steps/s with a trip of 240 made the
  belt skip at a load of about 204.

## Logs

`logs/` has the PC logs of the runs. They used an earlier build: ID 2 swept 1660 ↔ 150 at 150 steps/s, before
the dock was fitted. The current code docks between 1660 and 1080 at the slow settings instead.
