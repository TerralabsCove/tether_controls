#!/usr/bin/env python3
"""Dependency-free laptop console for the wireless Tether master."""

from __future__ import annotations

import argparse
import socket
import threading
import time


HELP = (
    "peers | reset N M | enable N M | disable N M | estop | zero N M | "
    "pos N M q dq | mit N M q dq tau kp kd | quit  "
    "(id 0: M=0; ids 1/2: M=0 big, 1 near-master, 2 far-end)"
)


def receiver(sock: socket.socket, stopped: threading.Event,
             telemetry_hz: float) -> None:
    sock.settimeout(0.5)
    last_telemetry: dict[str, float] = {}
    telemetry_period = 1.0 / telemetry_hz if telemetry_hz > 0 else None
    while not stopped.is_set():
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            return
        line = data.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("PONG"):
            continue
        if line.startswith("TEL "):
            if telemetry_period is None:
                continue
            fields = line.split()
            node = next((field for field in fields if field.startswith("id=")), "id=?")
            motor = next((field for field in fields if field.startswith("motor=")), "motor=?")
            stream = f"{node}/{motor}"
            now = time.monotonic()
            if now - last_telemetry.get(stream, -telemetry_period) < telemetry_period:
                continue
            last_telemetry[stream] = now
        print(line, flush=True)


def heartbeat(sock: socket.socket, destination: tuple[str, int],
              stopped: threading.Event) -> None:
    last_warning = 0.0
    while not stopped.wait(0.10):
        try:
            sock.sendto(b"ping", destination)
        except OSError as error:
            # A Wi-Fi route change can fail one UDP send while the socket and
            # receive path remain usable. Keep retrying instead of silently
            # killing the safety heartbeat for the rest of the session.
            now = time.monotonic()
            if now - last_warning >= 1.0:
                print(f"WARNING heartbeat send failed; retrying: {error}", flush=True)
                last_warning = now


def main() -> int:
    parser = argparse.ArgumentParser(description="Wireless console for Tether master id 0")
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=4210)
    parser.add_argument(
        "--telemetry-hz", type=float, default=2.0,
        help="maximum displayed telemetry rate per node; 0 hides telemetry (default: 2)",
    )
    args = parser.parse_args()
    if args.telemetry_hz < 0:
        parser.error("--telemetry-hz must be zero or positive")
    destination = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stopped = threading.Event()
    threading.Thread(
        target=receiver, args=(sock, stopped, args.telemetry_hz), daemon=True
    ).start()
    threading.Thread(target=heartbeat, args=(sock, destination, stopped), daemon=True).start()
    sock.sendto(b"ping", destination)
    print(f"Connected console target: udp://{args.host}:{args.port}")
    print(HELP)
    try:
        while True:
            line = input().strip()
            if not line:
                continue
            if line == "quit":
                sock.sendto(b"estop", destination)
                time.sleep(0.1)
                break
            if line == "help":
                print(HELP)
            sock.sendto(line.encode("ascii"), destination)
    except (EOFError, KeyboardInterrupt):
        try:
            sock.sendto(b"estop", destination)
            time.sleep(0.1)
        except OSError:
            pass
    finally:
        stopped.set()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
