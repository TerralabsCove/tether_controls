import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))

from bridge import ENABLE, ESTOP, MIT, POSITION_VELOCITY, RESET, EspCommand, SafetyController, parse_esp_command
from damiao_usbcan import CanFrame, crc8, encode_tx, extract_rx
from motor_protocol import (
    Limits, decode_float_parameter, disable_frame, parameter_read_frame,
    position_velocity_frame,
)


LIMITS = Limits(12.5, 20.0, 200.0)


class UsbCanTest(unittest.TestCase):
    def test_tx_frame_shape_and_crc(self):
        encoded = encode_tx(CanFrame(0x123, b"TETHER\x01\x00"))
        self.assertEqual(len(encoded), 30)
        self.assertEqual(encoded[:4], b"\x55\xAA\x1E\x03")
        self.assertEqual(encoded[-1], crc8(encoded[:-1] + b"\0"))

    def test_receive_resynchronizes(self):
        raw = bytearray(b"noise" + b"\xAA\x11\x08\x00\x00\x00\x16abcdefgh\x55")
        frames = extract_rx(raw)
        self.assertEqual(frames, [CanFrame(0x16, b"abcdefgh")])
        self.assertEqual(raw, bytearray())

    def test_receive_accepts_little_endian_id_firmware(self):
        raw = bytearray(b"\xAA\x11\x00\x16\x00\x00\x00abcdefgh\x55")
        self.assertEqual(extract_rx(raw), [CanFrame(0x16, b"abcdefgh")])

    def test_receive_skips_invalid_identifier(self):
        raw = bytearray(b"\xAA\x11\x40\xFF\xFF\xFF\xFFabcdefgh\x55")
        self.assertEqual(extract_rx(raw), [])

    def test_parameter_read_and_decode(self):
        request = parameter_read_frame(0x06, 22)
        self.assertEqual(request, CanFrame(0x7FF, b"\x06\x00\x33\x16\x00\x00\x00\x00"))
        import struct
        reply = CanFrame(0x16, b"\x06\x00\x33\x16" + struct.pack("<f", 20.0))
        self.assertEqual(decode_float_parameter(reply, 0x06, 22), 20.0)
        self.assertIsNone(decode_float_parameter(reply, 0x01, 22))


class SafetyTest(unittest.TestCase):
    def command(self, sequence, flags=0, mode=POSITION_VELOCITY):
        return EspCommand(sequence, flags, mode, 0.1, 0.25, 0, 0, 0)

    def test_parser(self):
        parsed = parse_esp_command("CMD,7,1,1,0.1,0.2,0,0,0")
        self.assertEqual(parsed.sequence, 7)
        self.assertIsNone(parse_esp_command("garbage"))

    def test_enable_needs_reset_and_recent_feedback(self):
        safety = SafetyController(6, LIMITS)
        self.assertEqual(safety.accept(self.command(1, RESET), 0.0), [])
        self.assertFalse(safety.estop)
        self.assertEqual(safety.accept(self.command(2, ENABLE), 0.1), [])
        safety.observe_feedback(0.11, 0)
        actions = safety.accept(self.command(3, ENABLE), 0.12)
        self.assertEqual([frame.arbitration_id for frame, _ in actions], [0x7FF, 6, 0x106])
        self.assertTrue(safety.enabled)

    def test_duplicate_watchdog_and_estop(self):
        safety = SafetyController(6, LIMITS)
        safety.accept(self.command(1, RESET), 0.0)
        safety.observe_feedback(0.01, 0)
        safety.accept(self.command(2, ENABLE), 0.02)
        self.assertEqual(safety.accept(self.command(2, ENABLE), 0.03), [])
        actions = safety.watchdog(0.121)
        self.assertEqual(actions[0][0], disable_frame(6))
        self.assertTrue(safety.estop)

    def test_clamps_position_velocity(self):
        frame = position_velocity_frame(6, 99, -99, LIMITS)
        import struct
        q, dq = struct.unpack("<ff", frame.data)
        self.assertEqual(q, 12.5)
        self.assertEqual(dq, 20.0)

    def test_master_estop_disables_motor(self):
        safety = SafetyController(6, LIMITS)
        safety.accept(self.command(1, RESET), 0.0)
        safety.observe_feedback(0.01, 0)
        safety.accept(self.command(2, ENABLE), 0.02)
        actions = safety.accept(self.command(3, ENABLE | ESTOP), 0.03)
        self.assertEqual(actions[0][0], disable_frame(6))
        self.assertTrue(safety.estop)
        self.assertFalse(safety.enabled)


if __name__ == "__main__":
    unittest.main()
