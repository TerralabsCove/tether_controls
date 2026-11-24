#!/usr/bin/env python3
"""Send commands to the Nano sts3215_led_test firmware at 115200 and print its replies.

    python bench_cli.py "scan" "rainbow 16 10" "swing 1 50"

Only printable text is kept.
Opening the port resets the Nano; the script waits for its "ready" line.
Needs pyserial (PlatformIO's Python has it: ~/.platformio/penv/bin/python).
"""
import argparse
import glob
import sys
import time

import serial


def text(b):
    return "".join(chr(c) for c in b if c == 10 or 32 <= c < 127)


def read_until_prompt(s, timeout):
    end, out = time.time() + timeout, b""
    while time.time() < end:
        out += s.read(4096)
        if text(out).rstrip().endswith(">"):
            break
    return text(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbserial*)")
    ap.add_argument("--timeout", type=float, default=90, help="seconds to wait per command")
    ap.add_argument("cmds", nargs="+")
    args = ap.parse_args()

    port = args.port or next(iter(sorted(glob.glob("/dev/cu.usbserial*"))), None)
    if not port:
        sys.exit("no /dev/cu.usbserial* port. Is the Nano on USB?")
    s = serial.Serial(port, 115200, timeout=0.2)
    print(read_until_prompt(s, 5).strip(), flush=True)
    for cmd in args.cmds:
        print(f">>> {cmd}", flush=True)
        s.reset_input_buffer()
        s.write(b"\n")  # wake byte: the Nano pauses its LED frames so the command is not dropped
        time.sleep(0.05)
        s.write((cmd + "\n").encode())
        out = read_until_prompt(s, args.timeout)
        lines = [l for l in out.splitlines() if l.strip() and l.strip() != cmd]
        print("\n".join(lines).rstrip().rstrip(">").rstrip(), flush=True)
    s.close()


if __name__ == "__main__":
    main()
