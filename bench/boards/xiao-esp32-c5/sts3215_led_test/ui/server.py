#!/usr/bin/env python3
"""Button UI for the sts3215_led_test firmware (XIAO C5 or Nano).

    ~/.platformio/penv/bin/python ui/server.py            then open http://127.0.0.1:8765

Holds the board's USB serial port open and serves ui/index.html. Only listens on 127.0.0.1.
One command runs at a time (the firmware answers "> " when it is done). Stop sends 's', which the
firmware checks inside every guarded move and every endurance cycle.
Needs pyserial (PlatformIO's Python has it).
"""
import argparse
import glob
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
QUIET = ("stat ",)  # commands the UI polls; kept out of the log


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/cu.usbserial*"))
    return ports[0] if ports else None


class Link:
    def __init__(self, port):
        self.want_port = port
        self.port = None
        self.ser = None
        self.err = ""
        self.lines = []  # (seq, time, text)
        self.seq = 0
        self.busy = False
        self.busy_cmd = ""
        self.busy_since = 0.0
        self.quiet = False
        self.lock = threading.Lock()
        self.open()
        threading.Thread(target=self.reader, daemon=True).start()

    def open(self):
        port = self.want_port or find_port()
        if not port:
            self.err = "no /dev/cu.usbmodem* or /dev/cu.usbserial* port"
            return
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = port, 115200, 0.1
            s.dtr = False
            s.rts = False
            s.open()
            self.ser, self.port, self.err = s, port, ""
            self.add(f"[connected to {port}]")
        except Exception as e:  # noqa: BLE001
            self.ser, self.err = None, str(e)

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser, self.busy = None, False

    def add(self, text):
        with self.lock:
            self.seq += 1
            self.lines.append((self.seq, time.time(), text))
            del self.lines[:-3000]

    def reader(self):
        buf = b""
        while True:
            if not self.ser:
                time.sleep(1)
                self.open()
                continue
            try:
                chunk = self.ser.read(512)
            except Exception as e:  # noqa: BLE001
                self.add(f"[serial error: {e}]")
                self.close()
                continue
            if self.busy and time.time() - self.busy_since > 900:
                self.busy = False
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = "".join(c for c in raw.decode(errors="replace") if c == "\t" or 32 <= ord(c) < 127).strip()
                if text == ">":
                    self.busy, self.quiet = False, False
                    continue
                if text:
                    if self.quiet and not text.startswith("STAT "):
                        self.quiet = False
                    self.add(text)

    def send(self, cmd):
        if not self.ser:
            return False, "not connected"
        if self.busy:
            return False, f"busy with '{self.busy_cmd}'"
        self.busy, self.busy_cmd, self.busy_since = True, cmd, time.time()
        self.quiet = cmd.startswith(QUIET)
        if not self.quiet:
            self.add(f">>> {cmd}")
        try:
            self.ser.write(b"\n")  # wake byte (the Nano pauses LED frames on it)
            time.sleep(0.05)
            self.ser.write((cmd + "\n").encode())
        except Exception as e:  # noqa: BLE001
            self.close()
            return False, str(e)
        return True, ""

    def stop(self):
        if self.ser:
            self.ser.write(b"s")
            self.add("[stop sent]")


def make_handler(link):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(HERE, "index.html"), "rb") as f:
                    return self.reply(200, f.read(), "text/html; charset=utf-8")
            if self.path.startswith("/api/state"):
                since = 0
                if "since=" in self.path:
                    try:
                        since = int(self.path.split("since=")[1].split("&")[0])
                    except ValueError:
                        pass
                with link.lock:
                    lines = [l for l in link.lines if l[0] > since]
                return self.reply(200, {
                    "connected": link.ser is not None, "port": link.port, "err": link.err,
                    "busy": link.busy, "busy_cmd": link.busy_cmd, "seq": link.seq, "lines": lines,
                })
            self.reply(404, {"error": "not found"})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                body = {}
            if self.path == "/api/cmd":
                cmd = str(body.get("cmd", "")).strip()
                if not cmd or len(cmd) > 60 or any(not (32 <= ord(c) < 127) for c in cmd):
                    return self.reply(400, {"ok": False, "error": "bad command"})
                ok, err = link.send(cmd)
                return self.reply(200 if ok else 409, {"ok": ok, "error": err})
            if self.path == "/api/stop":
                link.stop()
                return self.reply(200, {"ok": True})
            if self.path == "/api/reconnect":
                link.close()
                link.open()
                return self.reply(200, {"ok": link.ser is not None, "error": link.err})
            self.reply(404, {"error": "not found"})

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem* or /dev/cu.usbserial*)")
    ap.add_argument("--http", type=int, default=8765)
    a = ap.parse_args()
    link = Link(a.port)
    srv = ThreadingHTTPServer(("127.0.0.1", a.http), make_handler(link))
    print(f"servo UI on http://127.0.0.1:{a.http}  (serial: {link.port or link.err})", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
