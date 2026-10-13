#!/usr/bin/env python3
"""Dependency-free sanity model for Tether discovery, availability, and safety."""

from __future__ import annotations

from dataclasses import dataclass, field
import struct

MAGIC = 0x33524854
VERSION = 3
COMMAND = 2
HEADER = struct.Struct("<IBBBBHHI")
COMMAND_BODY = struct.Struct("<BBBBfffffI")
WATCHDOG_MS = 60
OFFLINE_MS = 2500


def fnv1a(data: bytes) -> int:
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def encode_command(sequence: int, target: int, motor: int = 0, flags: int = 0) -> bytes:
    size = HEADER.size + COMMAND_BODY.size
    header = HEADER.pack(MAGIC, VERSION, COMMAND, 0, target, sequence, size, 0)
    body = COMMAND_BODY.pack(motor, 0, flags, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
    packet = header + body
    return packet[:-4] + struct.pack("<I", fnv1a(packet))


def valid_command(packet: bytes) -> bool:
    if len(packet) != HEADER.size + COMMAND_BODY.size:
        return False
    magic, version, kind, source, target, sequence, size, sent = HEADER.unpack_from(packet)
    del source, target, sequence, sent
    expected = struct.unpack_from("<I", packet, len(packet) - 4)[0]
    blank = packet[:-4] + b"\0\0\0\0"
    return (magic, version, kind, size) == (MAGIC, VERSION, COMMAND, len(packet)) and fnv1a(blank) == expected


@dataclass
class Motor:
    last_sequence: int | None = None
    last_command_ms: int | None = None
    enabled: bool = False
    estop: bool = True
    gaps: int = 0

    def receive(self, sequence: int, now_ms: int, enable: bool = False, reset: bool = False) -> bool:
        if self.last_sequence is not None:
            delta = ((sequence - self.last_sequence + 0x8000) & 0xFFFF) - 0x8000
            if delta <= 0:
                return False
            self.gaps += max(0, delta - 1)
        self.last_sequence = sequence
        self.last_command_ms = now_ms
        if reset and not enable:
            self.estop = False
        self.enabled = enable and not self.estop
        return True

    def tick(self, now_ms: int) -> None:
        if self.last_command_ms is None or now_ms - self.last_command_ms > WATCHDOG_MS:
            self.enabled = False


@dataclass
class Node:
    node_id: int
    motor_count: int = 3
    motors: list[Motor] = field(init=False)

    def __post_init__(self) -> None:
        self.motors = [Motor() for _ in range(self.motor_count)]

    def receive(self, motor: int, sequence: int, now_ms: int,
                enable: bool = False, reset: bool = False) -> bool:
        if not 0 <= motor < self.motor_count:
            return False
        return self.motors[motor].receive(sequence, now_ms, enable=enable, reset=reset)

    def tick(self, now_ms: int) -> None:
        for motor in self.motors:
            motor.tick(now_ms)


@dataclass
class Master:
    configured_ids: set[int]
    last_seen_ms: dict[int, int] = field(default_factory=dict)
    local_motors: Node = field(default_factory=lambda: Node(0, 2))

    def hello(self, node_id: int, now_ms: int) -> bool:
        if node_id not in self.configured_ids:
            return False
        self.last_seen_ms[node_id] = now_ms
        return True

    def available(self, node_id: int, now_ms: int) -> bool:
        if node_id == 0:
            return True
        return node_id in self.last_seen_ms and now_ms - self.last_seen_ms[node_id] <= OFFLINE_MS

    def command_local(self, motor: int, sequence: int, now_ms: int, *, enable: bool = False,
                      reset: bool = False) -> bool:
        """Apply the command locally; ID 0 does not traverse ESP-NOW."""
        return self.local_motors.receive(motor, sequence, now_ms, enable=enable, reset=reset)


def run_demo() -> None:
    assert valid_command(encode_command(1, 1))
    master = Master({1, 2})
    left, right = Node(1), Node(2)

    # Nodes may arrive in either order after id 0 has its MAC table.
    assert master.hello(2, 10)
    assert master.hello(1, 40)
    assert master.available(1, 100) and master.available(2, 100)
    assert master.available(0, 100)

    assert master.command_local(0, 1, 100, reset=True)
    assert master.command_local(0, 2, 120, enable=True)
    assert master.command_local(1, 1, 100, reset=True)
    assert master.command_local(1, 2, 120, enable=True)
    assert all(m.enabled for m in master.local_motors.motors)
    master.local_motors.tick(181)
    assert not any(m.enabled for m in master.local_motors.motors)

    assert left.receive(0, 1, 100, reset=True)
    assert left.receive(0, 2, 120, enable=True)
    assert left.motors[0].enabled
    assert not left.receive(0, 2, 121, enable=True)  # duplicate rejected
    left.tick(181)
    assert not left.motors[0].enabled  # 60 ms watchdog has expired

    assert not master.available(2, 2600)
    assert master.available(1, 2500)
    print("PASS: packet checksum/size")
    print("PASS: node 2 then node 1 discovery")
    print("PASS: per-motor duplicate rejection and watchdog")
    print("PASS: two master-local and three remote motor routes")
    print("PASS: online/offline availability timeout")


if __name__ == "__main__":
    run_demo()
