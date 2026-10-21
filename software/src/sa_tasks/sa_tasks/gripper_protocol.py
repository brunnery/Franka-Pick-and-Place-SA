"""Serial protocol of the iris gripper firmware (docs/PLAN.md Section 6). Pure, no I/O.

PC -> board: "<id> <CMD> [args]\\n"; board -> PC: "<id> ACK", "<id> DONE [info]",
"<id> ERR <reason>", and the unsolicited lines "READY" and "WATCHDOG".
"""
from dataclasses import dataclass
from typing import Optional

MAX_DURATION_MS = 60000  # same limits as the firmware
MAX_SPEED = 255
ACK, DONE, ERR = "ACK", "DONE", "ERR"


@dataclass(frozen=True)
class Reply:
    id: Optional[int]   # None for READY / WATCHDOG
    kind: str           # ACK, DONE, ERR, READY, WATCHDOG
    text: str = ""      # DONE info or ERR reason


def _check(duration_ms: int, *speeds: int) -> None:
    if not 1 <= duration_ms <= MAX_DURATION_MS or not all(0 <= s <= MAX_SPEED for s in speeds):
        raise ValueError(f"duration {duration_ms} ms or speed {speeds} out of range")


def open_iris(id_: int, duration_ms: int, speed: int) -> str:
    _check(duration_ms, speed)
    return f"{id_} OPEN {duration_ms} {speed}\n"


def close_iris(id_: int, duration_ms: int, speed: int, hold_speed: int) -> str:
    _check(duration_ms, speed, hold_speed)
    return f"{id_} CLOSE {duration_ms} {speed} {hold_speed}\n"


def rotate(id_: int, direction: int, duration_ms: int, speed: int) -> str:
    _check(duration_ms, speed)
    if direction not in (1, -1):
        raise ValueError(f"direction must be +1 or -1, not {direction}")
    return f"{id_} ROT {direction:+d} {duration_ms} {speed}\n"


def ping(id_: int) -> str:
    return f"{id_} PING\n"


def stop(id_: int) -> str:
    return f"{id_} STOP\n"


def parse_reply(line: str) -> Optional[Reply]:
    """One line from the board; None if it is not part of the protocol (e.g. noise)."""
    parts = line.strip().split(maxsplit=2)
    if parts in (["READY"], ["WATCHDOG"]):
        return Reply(None, parts[0])
    if len(parts) < 2 or parts[1] not in (ACK, DONE, ERR) or not parts[0].isdigit():
        return None
    return Reply(int(parts[0]), parts[1], parts[2] if len(parts) > 2 else "")
