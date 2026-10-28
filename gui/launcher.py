"""Logic of the SA launcher (no Qt, no ROS): buttons from gui.yaml, the script each button
runs in its terminal, run status via pid files, the stud side (which taught observation
pose of poses.yaml is the stud's), the run timer and the rating of the last run record.

Temporary add-on for the lab sessions, not part of the thesis code (gui/README.md).
"""
import os
import re
import shlex
import shutil
import signal
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import yaml

OUTCOME_KEY = "outcome"
SIDES = ("left", "right")    # seen from the robot base: left = +y_B
NUT_POSE, STUD_POSE = "observe_nut", "observe_stud"


@dataclass(frozen=True)
class Entry:
    key: str            # unique, file-name safe
    label: str
    command: str        # placeholders filled in
    confirm: bool
    requires: Optional[str]
    tip: str
    uses: Tuple[str, ...] = ()   # hardware the command starts itself (robot, camera, arduino)


def load_config(path: Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def groups(cfg: dict) -> List[Tuple[str, List[Entry]]]:
    variables = cfg.get("variables") or {}
    result, seen = [], set()
    for group in cfg["groups"]:
        entries = []
        for item in group["entries"]:
            key = slug(f"{group['name']}_{item['label']}")
            if key in seen:
                raise ValueError(f"duplicate button '{item['label']}' in '{group['name']}'")
            seen.add(key)
            entries.append(Entry(key, item["label"], item["command"].format(**variables),
                                 bool(item.get("confirm", False)), item.get("requires"),
                                 item.get("tip", ""), tuple(item.get("uses") or ())))
        result.append((group["name"], entries))
    return result


def setup_files(cfg: dict, repo: Path) -> List[Path]:
    files = []
    for name in cfg["environment"]["setup_files"]:
        path = Path(os.path.expanduser(name))
        files.append(path if path.is_absolute() else repo / path)
    return files


def shell_script(entry: Entry, cfg: dict, repo: Path, pid_file: Path) -> str:
    """bash script run in the terminal: environment, the `before` commands, pid file while
    the command runs, exit code at the end, window stays open until Enter."""
    q = shlex.quote
    lines = ["#!/usr/bin/env bash"]
    for name, value in (cfg["environment"].get("exports") or {}).items():
        lines.append(f"export {name}={q(str(value))}")
    for path in setup_files(cfg, repo):
        lines.append(f"if [ -f {q(str(path))} ]; then source {q(str(path))}; "
                     f"else echo 'not found (skipped): {path}'; fi")
    lines += [f"cd {q(str(repo))} || exit 1",
              *(cfg["environment"].get("before") or []),
              f"echo $$ > {q(str(pid_file))}",
              f"trap {q('rm -f ' + q(str(pid_file)))} EXIT",
              # A handler (not ignore, which the children would inherit): Ctrl-C / Stop
              # reaches the command, this shell carries on so the log stays readable
              "trap ':' INT",
              f"printf '%s\\n\\n' {q('$ ' + entry.command)}",
              entry.command,
              "status=$?",
              f"rm -f {q(str(pid_file))}",
              f"printf '\\n[%s] finished, exit code %s. Enter closes this window.\\n' "
              f"{q(entry.label)} \"$status\"",
              "read -r _"]
    return "\n".join(lines) + "\n"


def terminal_command(cfg: dict, title: str, script: Path,
                     which: Callable[[str], Optional[str]] = shutil.which) -> List[str]:
    for template in cfg["terminals"]:
        if which(template[0]):
            return [part.format(title=title) for part in template] + ["bash", str(script)]
    names = ", ".join(t[0] for t in cfg["terminals"])
    raise RuntimeError(f"no terminal program found ({names})")


def read_pid(pid_file: Path) -> Optional[int]:
    try:
        return int(Path(pid_file).read_text().strip())
    except (OSError, ValueError):
        return None


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def running_pid(pid_file: Path, alive: Callable[[int], bool] = process_alive) -> Optional[int]:
    """pid of the terminal's bash if the command still runs; removes stale pid files."""
    pid = read_pid(pid_file)
    if pid is None:
        return None
    if alive(pid):
        return pid
    Path(pid_file).unlink(missing_ok=True)
    return None


def interrupt(pid: int) -> None:
    """Same as Ctrl-C in the terminal: SIGINT to the terminal's process group."""
    os.killpg(os.getpgid(pid), signal.SIGINT)


def conflicts(entry: Entry, running: List[Entry]) -> List[Tuple[str, str]]:
    """(label, resource) of running commands that start the same hardware as entry."""
    return [(other.label, resource) for other in running if other.key != entry.key
            for resource in entry.uses if resource in other.uses]


def stud_side(poses: dict) -> str:
    """Side of the stud's area: where observe_stud lies (y > 0 = left seen from the base)."""
    return "left" if poses["poses"][STUD_POSE]["position_mm"][1] > 0 else "right"


def area_xy(poses: dict) -> dict:
    return {name: poses["poses"][name]["position_mm"][:2] for name in (NUT_POSE, STUD_POSE)}


def swap_areas(text: str) -> str:
    """poses.yaml with the x/y of observe_nut and observe_stud exchanged: nut and stud swap
    their areas. z and the taught orientation stay with the name, so the shared observation
    height (robot.observation_height_from) does not change. Comment header kept."""
    data = yaml.safe_load(text)
    nut = data["poses"][NUT_POSE]["position_mm"]
    stud = data["poses"][STUD_POSE]["position_mm"]
    nut[:2], stud[:2] = stud[:2], nut[:2]
    header = "".join(line for line in text.splitlines(keepends=True) if line.startswith("#"))
    return header + yaml.safe_dump(data, sort_keys=False)


class RunTimer:
    """Run timer from the time marks sa_tasks publishes (tasks.yaml run_record): dicts with
    event, t_s (since the run started moving) and run (start time, identifies the run)."""

    def __init__(self, end_event: str, stop_event: Optional[str] = None):
        self.end_event = end_event
        self.stop_event = stop_event  # the clock stops here (cycle time), else at end_event
        self.run = None
        self.step = None
        self.marks = {}
        self.started_at = None      # local clock at t_s = 0
        self.finished = None        # the end event, once the run is over

    def feed(self, event: dict, now: float) -> None:
        if event.get("run") != self.run:  # a new run: start over
            self.run, self.step, self.marks, self.finished = event.get("run"), None, {}, None
            self.started_at = now - float(event["t_s"])
        self.step = event.get("step", self.step)
        self.marks[event["event"]] = float(event["t_s"])
        if event["event"] == self.end_event:
            self.finished = event

    def elapsed(self, now: float) -> Optional[float]:
        if self.started_at is None:
            return None
        if self.stop_event in self.marks:
            return self.marks[self.stop_event]
        return float(self.finished["t_s"]) if self.finished else now - self.started_at


def latest_record(repo: Path, pattern: str) -> Optional[Path]:
    """Newest run record matching pattern (file names start with the time stamp)."""
    paths = sorted(Path(repo).glob(pattern), key=lambda p: p.name)
    return paths[-1] if paths else None


STATUSES = ("yes", "partly", "no", "aborted", "unrated")


def run_status(record: dict) -> str:
    """yes / partly / no from the rating, else aborted (result not ok) or unrated."""
    screwed = (record.get(OUTCOME_KEY) or {}).get("screwed")
    if screwed in ("yes", "partly", "no"):
        return screwed
    return "unrated" if record.get("result") == "ok" else "aborted"


class RoundCounter:
    """Runs of one test round (record.round of sa_tasks) per status. A record is parsed again
    only when its file changed."""

    def __init__(self):
        self.cache = {}  # path -> (mtime, round, status)

    def count(self, paths, round_) -> dict:
        counts = dict.fromkeys(STATUSES, 0)
        for path in paths:
            try:
                mtime = path.stat().st_mtime
                if self.cache.get(path, (None,))[0] != mtime:
                    record = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                    self.cache[path] = (mtime, record.get("round"), run_status(record))
            except (OSError, yaml.YAMLError, AttributeError):
                continue
            _, record_round, status = self.cache[path]
            if record_round == round_:
                counts[status] += 1
        return counts


def set_outcome(text: str, outcome: dict) -> str:
    """Run record text with outcome as its last entry; a previous rating (always last, only
    this writes it) is replaced. Everything before stays byte for byte."""
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if line.startswith(OUTCOME_KEY + ":"):
            lines = lines[:i]
            break
    body = "".join(lines)
    if body and not body.endswith("\n"):
        body += "\n"
    return body + yaml.safe_dump({OUTCOME_KEY: outcome}, sort_keys=False, allow_unicode=True)
