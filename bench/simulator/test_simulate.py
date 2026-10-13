import unittest

from simulate import Master, Node, encode_command, valid_command


class TetherModelTest(unittest.TestCase):
    def test_packet_integrity(self):
        packet = encode_command(7, 2)
        self.assertTrue(valid_command(packet))
        damaged = bytearray(packet); damaged[10] ^= 1
        self.assertFalse(valid_command(bytes(damaged)))

    def test_discovery_order_and_timeout(self):
        master = Master({1, 2})
        self.assertTrue(master.hello(2, 10))
        self.assertTrue(master.hello(1, 20))
        self.assertTrue(master.available(2, 2500))
        self.assertFalse(master.available(2, 2511))

    def test_watchdog_and_sequences(self):
        node = Node(1)
        self.assertTrue(node.receive(0, 1, 0, reset=True))
        self.assertTrue(node.receive(0, 2, 20, enable=True))
        self.assertFalse(node.receive(0, 2, 21, enable=True))
        node.tick(81)
        self.assertFalse(node.motors[0].enabled)

    def test_master_local_motor_does_not_require_espnow(self):
        master = Master({1, 2})
        self.assertTrue(master.available(0, 0))
        for motor in (0, 1):
            self.assertTrue(master.command_local(motor, 1, 0, reset=True))
            self.assertTrue(master.command_local(motor, 2, 20, enable=True))
            self.assertTrue(master.local_motors.motors[motor].enabled)
        master.local_motors.tick(81)
        self.assertFalse(any(m.enabled for m in master.local_motors.motors))

    def test_motor_sequences_are_independent(self):
        node = Node(2)
        self.assertTrue(node.receive(0, 10, 0, reset=True))
        self.assertTrue(node.receive(1, 10, 0, reset=True))
        self.assertFalse(node.receive(0, 10, 1))
        self.assertTrue(node.receive(1, 11, 1))


if __name__ == "__main__":
    unittest.main()
