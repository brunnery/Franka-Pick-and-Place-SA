"""Iris gripper over USB serial (firmware arduino/iris_gripper, protocol in
gripper_protocol.py). Standard library only (termios), no ROS, no pyserial.

The board does all timing; every call sends one command and blocks until the board reports
DONE. A background thread sends PING every ping_period_s: the firmware switches all motors
off when the PC is silent for 2 s (watchdog), so the iris lets go when this process ends.
"""
import itertools
import logging
import os
import queue
import select
import termios
import threading
import tty
from typing import Callable, Dict, Optional

from sa_tasks import gripper_protocol as protocol

log = logging.getLogger("sa_tasks")

BAUD = {115200: termios.B115200, 57600: termios.B57600, 9600: termios.B9600}
READ_POLL_S = 0.1


class GripperError(RuntimeError):
    """The board refused a command, reported an error or did not answer in time."""


class SerialLink:
    """Raw serial port: write text, read whole lines."""

    def __init__(self, port: str, baud: int):
        if baud not in BAUD:
            raise ValueError(f"baud {baud} not supported, use one of {sorted(BAUD)}")
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = attrs[5] = BAUD[baud]
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buffer = b""

    def write(self, text: str) -> None:
        os.write(self.fd, text.encode("ascii"))

    def read_line(self, timeout_s: float) -> Optional[str]:
        """One line without the line end, or None if none arrived within timeout_s."""
        while b"\n" not in self.buffer:
            ready, _, _ = select.select([self.fd], [], [], timeout_s)
            if not ready:
                return None
            chunk = os.read(self.fd, 256)
            if not chunk:
                return None
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return line.decode("ascii", errors="replace").rstrip("\r")

    def close(self) -> None:
        os.close(self.fd)


class SerialGripper:
    """Gripper interface (interfaces.Gripper) on top of a line link to the board."""

    def __init__(self, link, cfg: dict):
        self.link = link
        self.cfg = cfg
        self.ids = itertools.count(1)
        self.write_lock = threading.Lock()
        self.waiting: Dict[int, queue.Queue] = {}
        self.stopped = threading.Event()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        # The board may just be booting (opening the port can reset it): wait for a PING
        try:
            self._command(lambda i: protocol.simple(i, "PING"), cfg["connect_timeout_s"], 0.0)
        except GripperError:
            self.shutdown()
            raise
        self.pinger = threading.Thread(target=self._ping_loop, daemon=True)
        self.pinger.start()
        log.info("gripper connected on %s", cfg["port"])

    def open(self) -> None:
        c = self.cfg["open"]
        log.info("gripper: open iris (%d ms, speed %d)", c["duration_ms"], c["speed"])
        self._timed(lambda i: protocol.open_iris(i, c["duration_ms"], c["speed"]),
                    c["duration_ms"])

    def close(self) -> None:
        c = self.cfg["close"]
        log.info("gripper: close iris (%d ms, speed %d), then hold with %d", c["duration_ms"],
                 c["speed"], c["hold_speed"])
        self._timed(lambda i: protocol.close_iris(i, c["duration_ms"], c["speed"],
                                                  c["hold_speed"]), c["duration_ms"])

    def rotate(self, direction: int, duration_ms: int, speed: int) -> None:
        log.info("gripper: rotate %+d (%d ms, speed %d)", direction, duration_ms, speed)
        self._timed(lambda i: protocol.rotate(i, direction, duration_ms, speed), duration_ms)

    def stop(self) -> None:
        self._command(lambda i: protocol.simple(i, "STOP"), self.cfg["ack_timeout_s"], 0.0)

    def shutdown(self) -> None:
        """Stop pinging and close the port; the board's watchdog switches the motors off."""
        self.stopped.set()
        for thread in (getattr(self, "pinger", None), self.reader):
            if thread is not None:
                thread.join(timeout=1.0)
        self.link.close()

    def _timed(self, build: Callable[[int], str], duration_ms: int) -> None:
        self._command(build, self.cfg["ack_timeout_s"],
                      duration_ms / 1000.0 + self.cfg["done_margin_s"])

    def _command(self, build: Callable[[int], str], first_timeout_s: float,
                 done_timeout_s: float) -> str:
        """Send one command; wait for ACK then DONE (timed commands) or DONE directly."""
        id_ = next(self.ids)
        text = build(id_)
        replies: queue.Queue = queue.Queue()
        self.waiting[id_] = replies
        try:
            with self.write_lock:
                self.link.write(text)
            reply = self._next(replies, first_timeout_s, text)
            if reply.kind == protocol.ACK:
                reply = self._next(replies, done_timeout_s, text)
            if reply.kind != protocol.DONE:
                raise GripperError(f"'{text.strip()}': {reply.kind} {reply.text}".strip())
            return reply.text
        finally:
            self.waiting.pop(id_, None)

    @staticmethod
    def _next(replies: queue.Queue, timeout_s: float, text: str) -> protocol.Reply:
        try:
            reply = replies.get(timeout=timeout_s)
        except queue.Empty:
            raise GripperError(f"'{text.strip()}': no answer within {timeout_s:.1f} s") from None
        if reply.kind == protocol.ERR:
            raise GripperError(f"'{text.strip()}': ERR {reply.text}")
        return reply

    def _read_loop(self) -> None:
        while not self.stopped.is_set():
            line = self.link.read_line(READ_POLL_S)
            if line is None:
                continue
            reply = protocol.parse_reply(line)
            if reply is None:
                log.debug("gripper: ignored line '%s'", line)
            elif reply.kind == "WATCHDOG":
                log.warning("gripper: watchdog fired, all motors off")
            elif reply.kind == "READY":
                log.info("gripper: board ready")
            elif reply.id in self.waiting:
                self.waiting[reply.id].put(reply)

    def _ping_loop(self) -> None:
        while not self.stopped.wait(self.cfg["ping_period_s"]):
            with self.write_lock:
                self.link.write(protocol.simple(next(self.ids), "PING"))


def connect(cfg: dict) -> SerialGripper:
    return SerialGripper(SerialLink(cfg["port"], cfg["baud"]), cfg)
