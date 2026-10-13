#!/usr/bin/env python3
"""ESP USB serial -> DAMIAO USB-CAN safety bridge."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import queue
import sys
import threading
import time

from damiao_usbcan import CanFrame, DamiaoUsbCan
from motor_protocol import (
    Limits, decode_feedback, decode_float_parameter, disable_frame,
    enable_frame, mit_frame, mode_frame, parameter_read_frame,
    position_velocity_frame, refresh_frame, zero_frame,
)

ENABLE, ESTOP, RESET, ZERO = 1, 2, 4, 8
IDLE, POSITION_VELOCITY, MIT = 0, 1, 2
PMAX, VMAX, TMAX = 21, 22, 23


@dataclass(frozen=True)
class EspCommand:
    sequence: int
    flags: int
    mode: int
    q: float
    dq: float
    tau: float
    kp: float
    kd: float


def parse_esp_command(line: str) -> EspCommand | None:
    fields = line.strip().split(",")
    if len(fields) != 9 or fields[0] != "CMD":
        return None
    try:
        command = EspCommand(int(fields[1]), int(fields[2]), int(fields[3]),
                             *(float(value) for value in fields[4:]))
    except ValueError:
        return None
    if not 0 <= command.sequence <= 0xFFFF or command.mode not in (IDLE, POSITION_VELOCITY, MIT):
        return None
    return command


class SafetyController:
    def __init__(self, motor_id: int, limits: Limits) -> None:
        self.motor_id = motor_id
        self.limits = limits
        self.estop = True
        self.enabled = False
        self.last_sequence: int | None = None
        self.last_command_at: float | None = None
        self.last_feedback_at: float | None = None
        self.previous_actions = 0

    def accept(self, command: EspCommand, now: float) -> list[tuple[CanFrame, float]]:
        if self.last_sequence is not None:
            delta = ((command.sequence - self.last_sequence + 0x8000) & 0xFFFF) - 0x8000
            if delta <= 0:
                return []
        self.last_sequence = command.sequence
        self.last_command_at = now
        actions: list[tuple[CanFrame, float]] = []
        if command.flags & ESTOP:
            self.estop = True
        edge_flags = command.flags & (RESET | ZERO)
        if (edge_flags & RESET) and not (self.previous_actions & RESET) and not (command.flags & ENABLE):
            self.estop = False
        if (edge_flags & ZERO) and not (self.previous_actions & ZERO) and not self.enabled and not self.estop:
            actions.append((zero_frame(self.motor_id), 0.0))
        self.previous_actions = edge_flags
        wants_enable = bool(command.flags & ENABLE) and command.mode != IDLE
        if self.estop or not wants_enable:
            if self.enabled:
                actions.append((disable_frame(self.motor_id), 0.0))
                self.enabled = False
            return actions
        if self.last_feedback_at is None or now - self.last_feedback_at > 0.5:
            return actions
        if not self.enabled:
            actions.append((mode_frame(self.motor_id, 1 if command.mode == MIT else 2), 0.02))
            actions.append((enable_frame(self.motor_id), 0.0))
            self.enabled = True
        if command.mode == POSITION_VELOCITY:
            actions.append((position_velocity_frame(self.motor_id, command.q, command.dq, self.limits), 0.0))
        elif command.mode == MIT:
            actions.append((mit_frame(self.motor_id, command.q, command.dq, command.tau,
                                      command.kp, command.kd, self.limits), 0.0))
        return actions

    def watchdog(self, now: float, timeout_s: float = 0.100) -> list[tuple[CanFrame, float]]:
        if self.last_command_at is None or now - self.last_command_at <= timeout_s:
            return []
        self.estop = True
        if not self.enabled:
            return []
        self.enabled = False
        return [(disable_frame(self.motor_id), 0.0)]

    def motor_fault(self) -> list[tuple[CanFrame, float]]:
        self.estop = True
        if not self.enabled:
            return []
        self.enabled = False
        return [(disable_frame(self.motor_id), 0.0)]

    def observe_feedback(self, now: float, error: int) -> list[tuple[CanFrame, float]]:
        self.last_feedback_at = now
        return self.motor_fault() if error >= 8 else []


def send_actions(adapter: DamiaoUsbCan, actions: list[tuple[CanFrame, float]]) -> None:
    for frame, delay_after in actions:
        adapter.send(frame)
        if delay_after:
            time.sleep(delay_after)


def console_reader(commands: queue.Queue[str]) -> None:
    while True:
        try:
            line = input().strip()
        except EOFError:
            line = "quit"
        commands.put(line)
        if line == "quit":
            return


def host_line(command: str) -> str | None:
    parts = command.split()
    if not parts: return None
    if parts[0] == "reset" and len(parts) == 1: return "SET,RESET\n"
    if parts[0] == "enable" and len(parts) == 1: return "SET,ENABLE,1\n"
    if parts[0] == "disable" and len(parts) == 1: return "SET,ENABLE,0\n"
    if parts[0] == "estop" and len(parts) == 1: return "SET,ESTOP\n"
    if parts[0] == "zero" and len(parts) == 1: return "SET,ZERO\n"
    if parts[0] == "pos" and len(parts) == 3: return f"SET,POS,{parts[1]},{parts[2]}\n"
    if parts[0] == "mit" and len(parts) == 6: return "SET,MIT," + ",".join(parts[1:]) + "\n"
    return None


def list_ports() -> None:
    import serial.tools.list_ports
    for port in serial.tools.list_ports.comports():
        print(f"{port.device:28} {port.description} vid={port.vid!r} pid={port.pid!r}")


def receive_for(adapter: DamiaoUsbCan, duration_s: float) -> list[CanFrame]:
    frames: list[CanFrame] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        frames.extend(adapter.receive())
        time.sleep(0.001)
    return frames


def scan_motors(adapter: DamiaoUsbCan, first_id: int = 1,
                last_id: int = 16) -> list[tuple[int, int, int]]:
    """Return (slave id, feedback CAN id, reply count) for responding motors."""
    if not 0 <= first_id <= last_id <= 0x7FF:
        raise ValueError("scan range must satisfy 0 <= first <= last <= 0x7ff")
    receive_for(adapter, 0.02)  # Discard adapter startup or stale traffic.
    hits: list[tuple[int, int, int]] = []
    for motor_id in range(first_id, last_id + 1):
        adapter.send(refresh_frame(motor_id))
        replies: dict[int, int] = {}
        for frame in receive_for(adapter, 0.04):
            if len(frame.data) != 8 or frame.data[2] in (0x33, 0x55):
                continue
            # DAMIAO feedback carries the slave ID's low nibble in byte zero.
            if (frame.data[0] & 0x0F) != (motor_id & 0x0F):
                continue
            replies[frame.arbitration_id] = replies.get(frame.arbitration_id, 0) + 1
        hits.extend((motor_id, feedback_id, count)
                    for feedback_id, count in sorted(replies.items()))
    return hits


def read_float_parameter(adapter: DamiaoUsbCan, motor_id: int,
                         register: int, retries: int = 4) -> float | None:
    for _ in range(retries):
        adapter.send(parameter_read_frame(motor_id, register))
        for frame in receive_for(adapter, 0.05):
            value = decode_float_parameter(frame, motor_id, register)
            if value is not None:
                return value
    return None


def inspect_motor(adapter: DamiaoUsbCan, motor_id: int) -> dict[str, float] | None:
    values: dict[str, float] = {}
    for name, register in (("PMAX", PMAX), ("VMAX", VMAX), ("TMAX", TMAX)):
        value = read_float_parameter(adapter, motor_id, register)
        if value is None:
            print(f"No reply for {name} (register {register}).", file=sys.stderr)
            return None
        values[name] = value
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="list serial devices and exit")
    diagnostic = parser.add_mutually_exclusive_group()
    diagnostic.add_argument(
        "--scan", action="store_true",
        help="safely probe motor IDs without enabling any motor",
    )
    diagnostic.add_argument(
        "--inspect", action="store_true",
        help="read PMAX, VMAX and TMAX from --motor-id",
    )
    parser.add_argument("--scan-start", type=lambda x: int(x, 0), default=1)
    parser.add_argument("--scan-end", type=lambda x: int(x, 0), default=16)
    parser.add_argument("--esp-port")
    parser.add_argument("--dm-port")
    parser.add_argument("--esp-baud", type=int, default=115200)
    parser.add_argument("--dm-baud", type=int, default=921600)
    parser.add_argument("--can-bitrate", type=int, default=1_000_000)
    parser.add_argument("--motor-id", type=lambda x: int(x, 0))
    parser.add_argument("--feedback-id", type=lambda x: int(x, 0))
    parser.add_argument(
        "--motor-slot", type=int, choices=(0, 1, 2), default=0,
        help="Tether motor selector: remote slot 0=big, 1=near-master, 2=far-end",
    )
    parser.add_argument("--pmax", type=float)
    parser.add_argument("--vmax", type=float)
    parser.add_argument("--tmax", type=float)
    parser.add_argument(
        "--wireless-node", action="store_true",
        help="take commands from the ESP-NOW node and disable the local command console",
    )
    args = parser.parse_args()
    if args.list:
        list_ports(); return 0
    if args.scan or args.inspect:
        if args.dm_port is None:
            parser.error("--scan and --inspect require --dm-port")
        if args.inspect and args.motor_id is None:
            parser.error("--inspect requires --motor-id")
        adapter = DamiaoUsbCan(args.dm_port, args.can_bitrate, args.dm_baud)
        try:
            if args.scan:
                print(f"Scanning slave IDs 0x{args.scan_start:02X}..0x{args.scan_end:02X} "
                      f"at {args.can_bitrate} bit/s; motors will not be enabled.")
                hits = scan_motors(adapter, args.scan_start, args.scan_end)
                if not hits:
                    print("No motors replied. Check power, bitrate, CAN wiring and termination.")
                    return 1
                by_slave: dict[int, list[int]] = {}
                for slave_id, feedback_id, count in hits:
                    by_slave.setdefault(slave_id, []).append(feedback_id)
                    print(f"slave=0x{slave_id:02X} feedback=0x{feedback_id:03X} "
                          f"replies={count}")
                for slave_id, feedback_ids in by_slave.items():
                    if len(feedback_ids) > 1:
                        print(f"WARNING: slave 0x{slave_id:02X} produced multiple feedback IDs; "
                              "check for duplicate motor IDs.")
                return 0
            values = inspect_motor(adapter, args.motor_id)
            if values is None:
                print("Inspection failed. Verify the motor ID, power, bitrate and bus.",
                      file=sys.stderr)
                return 1
            print(f"motor_id=0x{args.motor_id:02X} "
                  f"PMAX={values['PMAX']:.6g} rad "
                  f"VMAX={values['VMAX']:.6g} rad/s "
                  f"TMAX={values['TMAX']:.6g} Nm")
            print("bridge args: "
                  f"--motor-id 0x{args.motor_id:02X} "
                  f"--pmax {values['PMAX']:.6g} "
                  f"--vmax {values['VMAX']:.6g} "
                  f"--tmax {values['TMAX']:.6g}")
            return 0
        finally:
            adapter.close()
    required = ("esp_port", "dm_port", "motor_id", "feedback_id", "pmax", "vmax", "tmax")
    missing = [name.replace("_", "-") for name in required if getattr(args, name) is None]
    if missing:
        parser.error("missing required arguments: " + ", ".join("--" + name for name in missing))

    import serial
    limits = Limits(args.pmax, args.vmax, args.tmax)
    esp = serial.Serial(args.esp_port, args.esp_baud, timeout=0)
    adapter = DamiaoUsbCan(args.dm_port, args.can_bitrate, args.dm_baud)
    safety = SafetyController(args.motor_id, limits)
    command_queue: queue.Queue[str] = queue.Queue()
    if not args.wireless_node:
        threading.Thread(target=console_reader, args=(command_queue,), daemon=True).start()
    esp_buffer = bytearray()
    last_feedback_at: float | None = None
    last_print = 0.0
    next_refresh = 0.0
    running = True
    print("Bridge open. Motor is disabled and estop is latched.")
    if args.wireless_node:
        print(f"Wireless-node mode: commands for motor slot {args.motor_slot} "
              "come from laptop -> master -> ESP-NOW node.")
    else:
        print("Commands: reset | pos Q DQ | mit Q DQ TAU KP KD | enable | disable | zero | estop | status | quit")
        esp.write(b"SET,ESTOP\n")
    next_bridge_status = 0.0
    try:
        while running:
            now = time.monotonic()
            if args.wireless_node and now >= next_bridge_status:
                esp.write((f"BRIDGE,1,{args.motor_slot},{limits.position:.6f},{limits.velocity:.6f},"
                           f"{limits.torque:.6f}\n").encode("ascii"))
                next_bridge_status = now + 0.25
            while not command_queue.empty():
                user_command = command_queue.get_nowait()
                if user_command == "quit": running = False; break
                if user_command == "status":
                    age = None if last_feedback_at is None else 1000 * (now - last_feedback_at)
                    print(f"enabled={safety.enabled} estop={safety.estop} feedback_age_ms={age}")
                    continue
                line = host_line(user_command)
                if line is None: print("bad command")
                else: esp.write(line.encode("ascii"))

            waiting = esp.in_waiting
            if waiting: esp_buffer.extend(esp.read(waiting))
            while b"\n" in esp_buffer:
                raw, _, remainder = esp_buffer.partition(b"\n"); esp_buffer = bytearray(remainder)
                line = raw.decode("ascii", errors="replace").strip()
                command = parse_esp_command(line)
                if command is not None:
                    send_actions(adapter, safety.accept(command, now))
                elif line: print(f"ESP: {line}")

            for frame in adapter.receive():
                if frame.arbitration_id != args.feedback_id: continue
                feedback = decode_feedback(frame, limits); last_feedback_at = now
                send_actions(adapter, safety.observe_feedback(now, feedback.error))
                esp.write((f"TEL,{safety.last_sequence or 0},1,{int(safety.enabled)},"
                           f"{int(safety.estop)},{feedback.error},{feedback.position:.6f},"
                           f"{feedback.velocity:.6f},{feedback.torque:.6f},"
                           f"{feedback.mos_c},{feedback.rotor_c}\n").encode("ascii"))
                if now - last_print >= 0.1:
                    print(f"q={feedback.position:+.3f} dq={feedback.velocity:+.3f} "
                          f"tau={feedback.torque:+.2f} temp={feedback.mos_c}/{feedback.rotor_c} "
                          f"error=0x{feedback.error:X} enabled={safety.enabled}")
                    last_print = now
            if not safety.enabled and now >= next_refresh:
                adapter.send(refresh_frame(args.motor_id))
                next_refresh = now + 0.2
            send_actions(adapter, safety.watchdog(now))
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            try: adapter.send(disable_frame(args.motor_id))
            except Exception: pass
            time.sleep(0.01)
        adapter.close(); esp.close()
        print("Motor disable sent; bridge closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
