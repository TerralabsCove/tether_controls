"""Minimal classic-CAN transport for the DAMIAO serial USB-CAN adapter.

The framing follows DAMIAO's 30-byte transmit and 16-byte receive protocol.
Only Classical CAN data frames are exposed because that is all this bench uses.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import Callable


BITRATE_INDEX = {
    1_000_000: 0, 800_000: 1, 666_000: 2, 500_000: 3,
    400_000: 4, 250_000: 5, 200_000: 6, 125_000: 7,
    100_000: 8, 80_000: 9, 50_000: 10, 40_000: 11,
    20_000: 12, 10_000: 13, 5_000: 14,
}


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes
    extended: bool = False

    def __post_init__(self) -> None:
        if not 0 <= self.arbitration_id <= (0x1FFFFFFF if self.extended else 0x7FF):
            raise ValueError("CAN identifier out of range")
        if len(self.data) > 8:
            raise ValueError("Classical CAN data cannot exceed 8 bytes")


def crc8(data: bytes | bytearray) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8C if value & 1 else value >> 1
    return value


def encode_tx(frame: CanFrame) -> bytes:
    # DAMIAO frames reserve eight data bytes. Motor traffic always uses all 8.
    data = frame.data.rjust(8, b"\0")
    packet = bytearray(b"\x55\xAA\x1E\x03")
    packet.extend(struct.pack("<II", 1, 10))  # one send; 10 * 100 us interval
    packet.append(1 if frame.extended else 0)
    packet.extend(struct.pack("<I", frame.arbitration_id))
    packet.extend((0, len(frame.data), 0, 0))  # data frame, DLC, acceptance fields
    packet.extend(data)
    packet.append(0)
    packet[-1] = crc8(packet)
    if len(packet) != 30:
        raise AssertionError("incorrect DAMIAO transmit frame size")
    return bytes(packet)


def extract_rx(buffer: bytearray) -> list[CanFrame]:
    frames: list[CanFrame] = []
    while len(buffer) >= 16:
        try:
            start = buffer.index(0xAA)
        except ValueError:
            buffer.clear(); break
        if start:
            del buffer[:start]
        if len(buffer) < 16:
            break
        if buffer[15] != 0x55:
            del buffer[0]; continue
        raw = bytes(buffer[:16]); del buffer[:16]
        command, info = raw[1], raw[2]
        if command != 0x11:  # 0x12 is transmit confirmation.
            continue
        # The published adapter implementation returns the ID big-endian, but
        # some CDC-firmware variants return the same four bytes little-endian.
        # When the info byte does not describe the frame, prefer whichever
        # interpretation is a valid 11-bit ID. This also prevents unrelated or
        # malformed serial traffic from crashing the bridge.
        id_big = struct.unpack(">I", raw[3:7])[0]
        id_little = struct.unpack("<I", raw[3:7])[0]
        dlc = (info & 0x3F) if info else 8
        dlc = min(dlc, 8)
        if info & 0x40:
            if id_big > 0x1FFFFFFF:
                continue
            arbitration_id, extended = id_big, True
        elif id_big <= 0x7FF:
            arbitration_id, extended = id_big, False
        elif id_little <= 0x7FF:
            arbitration_id, extended = id_little, False
        elif info == 0 and id_big <= 0x1FFFFFFF:
            arbitration_id, extended = id_big, True
        elif info == 0 and id_little <= 0x1FFFFFFF:
            arbitration_id, extended = id_little, True
        else:
            continue
        frames.append(CanFrame(arbitration_id, raw[7:15][:dlc], extended))
    return frames


class DamiaoUsbCan:
    def __init__(self, port: str, can_bitrate: int = 1_000_000,
                 serial_baud: int = 921_600,
                 serial_factory: Callable | None = None) -> None:
        if can_bitrate not in BITRATE_INDEX:
            raise ValueError(f"unsupported CAN bitrate: {can_bitrate}")
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self.serial = serial_factory(port, baudrate=serial_baud, timeout=0)
        self.buffer = bytearray()
        self.serial.write(bytes((0x55, 0x05, BITRATE_INDEX[can_bitrate], 0xAA, 0x55)))
        time.sleep(0.2)

    def send(self, frame: CanFrame) -> None:
        self.serial.write(encode_tx(frame))

    def receive(self) -> list[CanFrame]:
        waiting = getattr(self.serial, "in_waiting", 0)
        if waiting:
            self.buffer.extend(self.serial.read(waiting))
        return extract_rx(self.buffer)

    def close(self) -> None:
        self.serial.close()
