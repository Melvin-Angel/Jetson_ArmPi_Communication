import os
import socket
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "protocol"))
from armpi_protocol import JsonLineSocket, ProtocolError, encode, make_message, validate_message


class ProtocolTest(unittest.TestCase):
    def test_round_trip_with_fragmentation(self):
        left, right = socket.socketpair()
        try:
            message = make_message("base_velocity", {"vx_mps": 0.1, "vy_mps": 0, "wz_radps": -0.2}, "one")
            raw = encode(message)
            left.sendall(raw[:9])
            left.sendall(raw[9:])
            self.assertEqual(JsonLineSocket(right).recv_messages(), [message])
        finally:
            left.close()
            right.close()

    def test_rejects_bad_arm_pose(self):
        message = make_message("arm_pose", {"position_m": [0, 1], "pitch_deg": 0, "duration_s": 1})
        with self.assertRaises(ProtocolError):
            validate_message(message)

    def test_gripper_requires_operation(self):
        with self.assertRaises(ProtocolError):
            validate_message(make_message("gripper", {}))


if __name__ == "__main__":
    unittest.main()
