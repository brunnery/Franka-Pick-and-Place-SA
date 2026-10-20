"""Serial protocol of the iris gripper firmware (docs/PLAN.md Section 6). Pure, no I/O.

PC -> board: "<id> <CMD> [args]\\n"; board -> PC: "<id> ACK", "<id> DONE [info]",
"<id> ERR <reason>", and the unsolicited lines "READY" and "WATCHDOG".
"""
from dataclasses import dataclass
from typing import Optional

MAX_DURATION_MS = 60000  # same limit as the firmware
MAX_SPEED = 255
ACK, DONE, ERR = "ACK", "DONE", "ERR"


@dataclass(frozen=True)
class Reply:
    id: Optional[int]   # None for READY / WATCHDOG
    kind: str           # ACK, DONE, ERR, READY, WATCHDOG
    text: str = ""      # DONE info or ERR reason


def _duration(ms: int) -> int:
    if not 1 <= int(ms) <= MAX_DURATION_MS:
        raise ValueError(f"duration {ms} ms outside 1..{MAX_DURATION_MS}")
    return int(ms)


def _speed(speed: int) -> int:
    if not 0 <= int(speed) <= MAX_SPEED:
        raise ValueError(f"speed {speed} outside 0..{MAX_SPEED}")
    return int(speed)


def open_iris(id_: int, duration_ms: int, speed: int) -> str:
    return f"{id_} OPEN {_duration(duration_ms)} {_speed(speed)}\n"


def close_iris(id_: int, duration_ms: int, speed: int, hold_speed: int) -> str:
    return f"{id_} CLOSE {_duration(duration_ms)} {_speed(speed)} {_speed(hold_speed)}\n"


def rotate(id_: int, direction: int, duration_ms: int, speed: int) -> str:
    if direction not in (1, -1):
        raise ValueError(f"direction must be +1 or -1, not {direction}")
    return f"{id_} ROT {direction:+d} {_duration(duration_ms)} {_speed(speed)}\n"


def simple(id_: int, command: str) -> str:
    """STOP, PING or STATE."""
    if command not in ("STOP", "PING", "STATE"):
        raise ValueError(f"unknown command {command}")
    return f"{id_} {command}\n"


def parse_reply(line: str) -> Optional[Reply]:
    """One line from the board; None if it is not part of the protocol (e.g. noise)."""
    parts = line.strip().split(maxsplit=2)
    if not parts:
        return None
    if parts[0] in ("READY", "WATCHDOG") and len(parts) == 1:
        return Reply(None, parts[0])
    if len(parts) < 2 or parts[1] not in (ACK, DONE, ERR):
        return None
    try:
        id_ = int(parts[0])
    except ValueError:
        return None
    return Reply(id_, parts[1], parts[2] if len(parts) > 2 else "")
