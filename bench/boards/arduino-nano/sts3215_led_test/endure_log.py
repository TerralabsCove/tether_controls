#!/usr/bin/env python3
"""Start the firmware's endurance loop and log it.

    python endure_log.py --log endure.log [--args "50 25 16"]

Writes every line with a wall-clock timestamp. Exits when the Nano prints ENDURE STOP, when it has been
silent for --silence seconds (hung, reset or dead), or when the port goes away. Ctrl-C or SIGTERM sends 's'
so the Nano stops the servos cleanly and turns their torque off.
Needs pyserial (PlatformIO's Python has it: ~/.platformio/penv/bin/python).
"""
import argparse
import glob
import signal
import sys
import time

import serial

stop = False


def on_signal(*_):
    global stop
    stop = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--log", default="endure.log")
    ap.add_argument("--args", default="", help='endure arguments, e.g. "50 25 16"')
    ap.add_argument("--silence", type=float, default=120)
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    port = a.port or next(iter(sorted(glob.glob("/dev/cu.usbserial*"))), None)
    if not port:
        sys.exit("no /dev/cu.usbserial* port")
    s = serial.Serial(port, 115200, timeout=0.5)
    log = open(a.log, "a", buffering=1)

    def emit(line):
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        log.write(f"{stamp} {line}\n")
        print(f"{stamp} {line}", flush=True)

    # wait for the ready prompt (opening the port resets the Nano)
    buf, end = b"", time.time() + 6
    while time.time() < end and not buf.rstrip().endswith(b">"):
        buf += s.read(256)
    s.write(b"\n")  # wake byte: the Nano pauses its LED frames so the command is not dropped
    time.sleep(0.05)
    s.write(f"endure {a.args}\n".strip().encode() + b"\n")
    emit(f"host: started 'endure {a.args}' on {port}")

    pending, last_rx, sent_stop = b"", time.time(), 0.0
    while True:
        if stop and time.time() - sent_stop > 2:
            s.write(b"s")
            sent_stop = time.time()
            if sent_stop - last_rx > 60:
                emit("host: no reply to stop, exiting")
                return 1
        try:
            chunk = s.read(256)
        except serial.SerialException as e:
            emit(f"host: serial error ({e}). Nano unplugged or dead?")
            return 2
        if chunk:
            last_rx = time.time()
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if text and text != ">":
                    emit(text)
                if text.startswith("ENDURE STOP"):
                    return 0
        elif time.time() - last_rx > a.silence:
            emit(f"host: Nano silent for {a.silence:.0f} s. Hung, reset, or dead.")
            return 3


if __name__ == "__main__":
    sys.exit(main())
