import json
import unittest

from raspberry_pi.hardware.serial_comm import SerialComm, SerialConfig


class _FakeSerial:
    def __init__(self):
        self.is_open = True
        self.writes: list[bytes] = []
        self.reads: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def readline(self) -> bytes:
        if not self.reads:
            return b""
        return self.reads.pop(0)

    def close(self) -> None:
        self.is_open = False

    def open(self) -> None:
        self.is_open = True


class TestSerialComm(unittest.TestCase):
    def setUp(self):
        self.logs: list[str] = []
        self.fake = _FakeSerial()
        self.comm = SerialComm(
            config=SerialConfig(port="/dev/null", baudrate=115200),
            serial_port=self.fake,
            logger=self.logs.append,
        )

    def test_send_move_serializes_single_line_json(self):
        self.comm.send_move(0.2, -0.1)
        self.assertEqual(len(self.fake.writes), 1)

        payload = self.fake.writes[0].decode("utf-8")
        self.assertTrue(payload.endswith("\n"))
        data = json.loads(payload.strip())
        self.assertEqual(data, {"cmd": "move", "v": 0.2, "w": -0.1})

    def test_rejects_unsupported_cmd(self):
        with self.assertRaises(ValueError):
            self.comm.send_message({"cmd": "gimbal", "pan": 0.0, "tilt": 0.0})

    def test_read_message_parses_json_object(self):
        self.fake.reads.append(b'{"status":"ok","battery":87}\n')
        msg = self.comm.read_message()
        self.assertEqual(msg, {"status": "ok", "battery": 87})

    def test_read_message_rejects_non_object_json(self):
        self.fake.reads.append(b'[1,2,3]\n')
        with self.assertRaises(ValueError):
            self.comm.read_message()

    def test_log_format_contains_direction_and_message(self):
        self.comm.send_stop()
        self.assertTrue(self.logs)
        log_line = self.logs[0]
        self.assertIn("[TX]", log_line)
        self.assertIn('"cmd":"move"', log_line)


if __name__ == "__main__":
    unittest.main()
