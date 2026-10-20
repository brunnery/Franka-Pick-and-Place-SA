import queue
import threading
import time

import pytest

from sa_tasks import gripper_protocol as protocol
from sa_tasks.serial_gripper import GripperError, SerialGripper

CFG = {"port": "fake", "connect_timeout_s": 1.0, "ack_timeout_s": 0.3, "done_margin_s": 0.3,
       "ping_period_s": 0.05, "open": {"duration_ms": 20, "speed": 200},
       "close": {"duration_ms": 20, "speed": 200, "hold_speed": 120}}


def test_protocol_formats_checks_and_parses():
    assert protocol.close_iris(4, 1500, 200, 120) == "4 CLOSE 1500 200 120\n"
    assert protocol.rotate(5, -1, 8000, 255) == "5 ROT -1 8000 255\n"
    with pytest.raises(ValueError):
        protocol.open_iris(1, 0, 200)
    assert protocol.parse_reply("7 DONE iris=holding") == protocol.Reply(7, "DONE", "iris=holding")
    assert protocol.parse_reply("WATCHDOG") == protocol.Reply(None, "WATCHDOG")
    assert protocol.parse_reply("x ACK") is None


class FakeBoard:
    """Line link that answers like the firmware (timed commands: ACK, then DONE)."""

    def __init__(self, refuse=None):
        self.lines, self.sent, self.refuse, self.closed = queue.Queue(), [], refuse, False

    def write(self, text):
        self.sent.append(text.strip())
        id_, command = text.split()[:2]
        if command == self.refuse:
            self.lines.put(f"{id_} ERR args")
        elif command == "PING":
            self.lines.put(f"{id_} DONE")
        else:
            self.lines.put(f"{id_} ACK")
            threading.Timer(0.02, self.lines.put, [f"{id_} DONE"]).start()

    def read_line(self, timeout_s):
        try:
            return self.lines.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


def test_close_waits_for_done_and_pings_keep_the_board_alive():
    board = FakeBoard()
    gripper = SerialGripper(board, CFG)
    gripper.close()
    time.sleep(0.2)
    gripper.shutdown()
    assert "CLOSE 20 200 120" in [line.split(maxsplit=1)[1] for line in board.sent]
    assert sum("PING" in line for line in board.sent) >= 3
    assert board.closed


def test_an_error_from_the_board_raises():
    gripper = SerialGripper(FakeBoard(refuse="CLOSE"), CFG)
    with pytest.raises(GripperError, match="ERR args"):
        gripper.close()
    gripper.shutdown()
