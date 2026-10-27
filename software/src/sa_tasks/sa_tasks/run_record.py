"""Additions to the run record for the evaluation: time marks of a run and which settings it
ran with. No ROS (task_node publishes the marks for the GUI timer).
"""
import subprocess
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

GIT_TIMEOUT_S = 5.0


class Timeline:
    """Seconds since the start at named events (record: timeline_s). A name used twice gets
    a suffix (_2, _3, ...). Every mark is also handed to listener(name, t_s)."""

    def __init__(self, listener: Optional[Callable[[str, float], None]] = None,
                 clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self.start = clock()
        self.listener = listener
        self.marks: Dict[str, float] = {}

    def elapsed(self) -> float:
        return round(self.clock() - self.start, 2)

    def mark(self, name: str) -> float:
        key, n = name, 1
        while key in self.marks:
            n += 1
            key = f"{name}_{n}"
        t = self.marks[key] = self.elapsed()
        if self.listener is not None:
            self.listener(key, t)
        return t


def config_values(cfg: dict, keys: List[str]) -> dict:
    """{dotted key: value} of the listed tasks.yaml entries (KeyError if one is missing)."""
    values = {}
    for key in keys:
        value = cfg
        for part in key.split("."):
            value = value[part]
        values[key] = value
    return values


def git_state(repo: Path) -> dict:
    """Commit of the repository and the tracked files changed since (None if git fails)."""
    def git(*args) -> Optional[str]:
        try:
            result = subprocess.run(["git", "-C", str(repo), *args], capture_output=True,
                                    text=True, timeout=GIT_TIMEOUT_S)
        except (OSError, subprocess.SubprocessError):
            return None
        return result.stdout.strip() if result.returncode == 0 else None
    status = git("status", "--porcelain", "--untracked-files=no")
    return {"commit": git("rev-parse", "--short", "HEAD"),
            "changed_files": None if status is None else [line[3:] for line in
                                                          status.splitlines()]}
