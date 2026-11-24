#!/usr/bin/env python3
"""Send commands to the sts3215_led_test firmware and print the replies.

    python bench_cli.py "scan 25" "go 2 -10 30" "led 0"
    python bench_cli.py --port /dev/cu.usbmodem101 "scan 25 1000000 253"

Needs pyserial (PlatformIO's Python has it: ~/.platformio/penv/bin/python).
"""
import argparse
import glob
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--timeout", type=float, default=60, help="seconds to wait per command")
    ap.add_argument("cmds", nargs="+")
    args = ap.parse_args()

    port = args.port or next(iter(sorted(glob.glob("/dev/cu.usbmodem*"))), None)
    if not port:
        sys.exit("no /dev/cu.usbmodem* port. Is the C5 on USB?")
    s = serial.Serial(port, 115200, timeout=0.3)
    for cmd in args.cmds:
        print(f">>> {cmd}", flush=True)
        s.write((cmd + "\n").encode())
        end, out = time.time() + args.timeout, b""
        while time.time() < end:
            out += s.read(4096)
            if out.rstrip().endswith(b">"):  # firmware prints "> " when a command is done
                break
        print(out.decode(errors="replace").rstrip().rstrip(">").rstrip(), flush=True)
    s.close()


if __name__ == "__main__":
    main()
