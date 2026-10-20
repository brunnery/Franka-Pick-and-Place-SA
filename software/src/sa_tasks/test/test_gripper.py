import queue
import threading
import time

import pytest

from sa_tasks import gripper_protocol as protocol
from sa_tasks.serial_gripper import GripperError, SerialGripper

CFG = {"port": "fake", "baud": 115200, "connect_timeout_s": 1.0, "ack_timeout_s": 0.3,
       "done_margin_s": 0.3, "ping_period_s": 0.05,
       "open": {"duration_ms": 20, "speed": 200},
       "close": {"duration_ms": 20, "speed": 200, "hold_speed": 120}}


def test_commands_are_formatted_as_the_firmware_expects():
    assert protocol.open_iris(3, 1500, 200) == "3 OPEN 1500 200\n"
    assert protocol.close_iris(4, 1500, 200, 120) == "4 CLOSE 1500 200 120\n"
    assert protocol.rotate(5, -1, 8000, 255) == "5 ROT -1 8000 255\n"
    assert protocol.simple(6, "PING") == "6 PING\n"


@pytest.mark.parametrize("build", [
    lambda: protocol.open_iris(1, 0, 200),
    lambda: protocol.open_iris(1, 60001, 200),
    lambda: protocol.close_iris(1, 100, 200, 256),
    lambda: protocol.rotate(1, 0, 100, 200),
    lambda: protocol.simple(1, "RESET"),
])
def test_invalid_commands_are_refused_before_sending(build):
    with pytest.raises(ValueError):
        build()


def test_replies_are_parsed():
    assert protocol.parse_reply("7 ACK\r\n") == protocol.Reply(7, "ACK")
    assert protocol.parse_reply("7 DONE iris=holding rot=off") == \
        protocol.Reply(7, "DONE", "iris=holding rot=off")
    assert protocol.parse_reply("8 ERR args") == protocol.Reply(8, "ERR", "args")
    assert protocol.parse_reply("READY") == protocol.Reply(None, "READY")
    assert protocol.parse_reply("WATCHDOG") == protocol.Reply(None, "WATCHDOG")
    for noise in ("", "garbage", "x ACK", "3 HELLO"):
        assert protocol.parse_reply(noise) is None


class FakeBoard:
    """Line link that answers like the firmware (timed commands: ACK, then DONE)."""

    def __init__(self, refuse=None, silent=False):
        self.lines = queue.Queue()
        self.sent = []
        self.refuse = refuse
        self.silent = silent
        self.closed = False

    def write(self, text):
        self.sent.append(text.strip())
        id_, command = text.split()[:2]
        if self.silent:
            return
        if command == self.refuse:
            self.lines.put(f"{id_} ERR args")
        elif command in ("OPEN", "CLOSE", "ROT"):
            self.lines.put(f"{id_} ACK")
            threading.Timer(0.02, self.lines.put, [f"{id_} DONE"]).start()
        else:
            self.lines.put(f"{id_} DONE")

    def read_line(self, timeout_s):
        try:
            return self.lines.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


def commands(board):
    return [line.split()[1] for line in board.sent]


def test_close_waits_for_done_and_pings_keep_the_board_alive():
    board = FakeBoard()
    gripper = SerialGripper(board, CFG)
    gripper.close()
    time.sleep(0.2)
    gripper.shutdown()
    assert [line.split(maxsplit=1)[1] for line in board.sent if "CLOSE" in line] == \
        ["CLOSE 20 200 120"]
    assert commands(board).count("PING") >= 3  # connect + keep-alive
    assert board.closed


def test_open_rotate_and_stop_are_sent():
    board = FakeBoard()
    gripper = SerialGripper(board, CFG)
    gripper.open()
    gripper.rotate(-1, 30, 255)
    gripper.stop()
    gripper.shutdown()
    sent = [line for line in board.sent if "PING" not in line]
    assert [line.split(maxsplit=1)[1] for line in sent] == \
        ["OPEN 20 200", "ROT -1 30 255", "STOP"]


def test_an_error_from_the_board_raises():
    gripper = SerialGripper(FakeBoard(refuse="CLOSE"), CFG)
    with pytest.raises(GripperError, match="ERR args"):
        gripper.close()
    gripper.shutdown()


def test_a_silent_board_fails_on_connect_and_the_port_is_closed():
    board = FakeBoard(silent=True)
    with pytest.raises(GripperError, match="no answer"):
        SerialGripper(board, CFG)
    assert board.closed
