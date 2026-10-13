"""DAMIAO Classical-CAN frame encoding used by the temporary bridge."""

from __future__ import annotations

from dataclasses import dataclass
import struct

from damiao_usbcan import CanFrame


@dataclass(frozen=True)
class Limits:
    position: float
    velocity: float
    torque: float

    def __post_init__(self) -> None:
        if self.position <= 0 or self.velocity <= 0 or self.torque <= 0:
            raise ValueError("all motor limits must be positive")


@dataclass(frozen=True)
class Feedback:
    position: float
    velocity: float
    torque: float
    error: int
    mos_c: int
    rotor_c: int


def control_frame(motor_id: int, byte: int) -> CanFrame:
    return CanFrame(motor_id, b"\xFF" * 7 + bytes((byte,)))


def enable_frame(motor_id: int) -> CanFrame: return control_frame(motor_id, 0xFC)
def disable_frame(motor_id: int) -> CanFrame: return control_frame(motor_id, 0xFD)
def zero_frame(motor_id: int) -> CanFrame: return control_frame(motor_id, 0xFE)
def refresh_frame(motor_id: int) -> CanFrame: return control_frame(motor_id, 0xCC)


def parameter_read_frame(motor_id: int, register: int) -> CanFrame:
    """Request one DAMIAO register without changing motor state."""
    if not 0 <= motor_id <= 0xFFFF:
        raise ValueError("motor identifier out of range")
    if not 0 <= register <= 0xFF:
        raise ValueError("register out of range")
    return CanFrame(0x7FF, struct.pack("<HBBI", motor_id, 0x33, register, 0))


def decode_float_parameter(frame: CanFrame, motor_id: int,
                           register: int) -> float | None:
    """Decode a matching float register reply; ignore unrelated CAN traffic."""
    if len(frame.data) != 8 or frame.data[2] not in (0x33, 0x55):
        return None
    reply_motor_id = frame.data[0] | (frame.data[1] << 8)
    if reply_motor_id != motor_id or frame.data[3] != register:
        return None
    return struct.unpack("<f", frame.data[4:8])[0]


def mode_frame(motor_id: int, mode: int) -> CanFrame:
    return CanFrame(0x7FF, struct.pack("<HBBBBBB", motor_id, 0x55, 10, mode, 0, 0, 0))


def position_velocity_frame(motor_id: int, position: float, velocity: float,
                            limits: Limits) -> CanFrame:
    position = max(-limits.position, min(position, limits.position))
    velocity = max(0.0, min(abs(velocity), limits.velocity))
    return CanFrame(motor_id + 0x100, struct.pack("<ff", position, velocity))


def _pack(value: float, minimum: float, maximum: float, bits: int) -> int:
    value = max(minimum, min(value, maximum))
    return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def _unpack(value: int, minimum: float, maximum: float, bits: int) -> float:
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def mit_frame(motor_id: int, position: float, velocity: float, torque: float,
              kp: float, kd: float, limits: Limits) -> CanFrame:
    p = _pack(position, -limits.position, limits.position, 16)
    v = _pack(velocity, -limits.velocity, limits.velocity, 12)
    t = _pack(torque, -limits.torque, limits.torque, 12)
    pg = _pack(kp, 0, 500, 12)
    dg = _pack(kd, 0, 5, 12)
    data = bytes((p >> 8, p & 0xFF, v >> 4,
                  ((v & 0xF) << 4) | (pg >> 8), pg & 0xFF, dg >> 4,
                  ((dg & 0xF) << 4) | (t >> 8), t & 0xFF))
    return CanFrame(motor_id, data)


def decode_feedback(frame: CanFrame, limits: Limits) -> Feedback:
    if len(frame.data) != 8:
        raise ValueError("DAMIAO feedback must contain eight bytes")
    data = frame.data
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    return Feedback(_unpack(p, -limits.position, limits.position, 16),
                    _unpack(v, -limits.velocity, limits.velocity, 12),
                    _unpack(t, -limits.torque, limits.torque, 12),
                    data[0] >> 4, data[6], data[7])
