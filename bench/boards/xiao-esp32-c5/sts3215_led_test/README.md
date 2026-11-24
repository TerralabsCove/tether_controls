# STS3215 + LED ring bench test (XIAO ESP32-C5)

A bench sketch, not deployment firmware. It drives FEETECH STS3215 12 V serial bus servos
straight from one XIAO ESP32-C5 GPIO, with no adapter board, and tests a WS2812B ring.

Results from 2026-11-24 are summarized in the tether_context note
`notes/2026-11-24-john-sts3215-c5-bench.md`.

## Wiring used on 2026-11-24

| Signal | XIAO pin | GPIO |
|---|---|---|
| Servo data (STS3215 bus, both servos daisy-chained) | D2 | 25 |
| LED ring DIN | D1 | 0 |
| GND (shared with the servo battery and the buck) | GND | |

The servos run off the 3S pack directly. The ring runs off the 5 V buck.

**Power the C5 from USB or from the buck, never both.** A C5 running on the buck's 5 V while it was
also plugged into the laptop got hot and died. To flash or test from the laptop, disconnect the buck's +
lead from the C5's 5V pad and keep every ground connected. Unplug USB before you reconnect the buck's +.

## Build and flash

```bash
pio run -e xiao_c5 -t upload
```

Uses the pioarduino platform (Arduino-ESP32 3.3.x) and Adafruit NeoPixel. There is also an `xiao_s3`
env. It built and flashed, but the S3 board it ran on stopped responding, so it is untested.

## Commands

Send over USB serial at 115200, or use `bench_cli.py`:

```bash
~/.platformio/penv/bin/python bench_cli.py "scan 25" "go 2 -10 30"
```

| Command | What it does |
|---|---|
| `scan <gpio> [baud] [maxid]` | Ping IDs 0..maxid (default 30). Read only. Prints position, mode, torque, voltage, temperature, limits. |
| `nudge <deg> [id] [tq%]` | +deg, back, -deg, back at about 13 deg/s. Max 45. |
| `go <id> <deg> [tq%]` | One move by deg (sign = direction), then back. Max 90. |
| `led <gpio> [n]` | Red, green, blue, chase at about 10 % brightness, then dim blue. `off` turns it off. |
| `raw <id>` | Ping one ID (254 = broadcast) and hex-dump the wire. |
| `line <gpio>` | Idle level with pull-down, then pull-up. Shows whether anything drives the pin. |
| `drive <0\|1>` | 1 (default): push-pull while sending. 0: open drain only. |

Every `nudge` and `go`:

- caps torque (register 48, RAM) at `tq%`, default 30 %, and restores the old value afterwards;
- holds the current position before enabling torque, so the servo does not jump to an old goal;
- stops and goes back to the start if the load reaches 80 % of the cap, or if the joint stops
  moving for 0.4 s;
- turns torque back off at the end if it was off at the start.

## How the one-wire bus is driven

TX and RX share the pad. The pad is open drain with pull-up while listening. While sending it is
push-pull. It switches back to open drain as soon as the UART reports the last stop bit is out.

With open drain only, the servo never answered at any ID or baud rate. Our own bytes echoed back clean.
Pushing the line high fixed it at once. The likely cause is a weak pull-up in the servo giving slow
rising edges at 1 Mbit/s. That is assumed, not measured with a scope.

The servo's data line idled at about 3.07 V against the servo's own GND, so 3.3 V logic is fine. The
FEETECH spec sheet for the 7.4 V STS3215 gives high 2-5 V and low 0-0.45 V. No 12 V sheet was found.
