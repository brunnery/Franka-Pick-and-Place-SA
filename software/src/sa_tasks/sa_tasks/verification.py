"""Verification of the target computation: hand-guided truth vs. computed nut position.

  ros2 run sa_tasks compare_runs [--since 2026-10-18T11:00] [--truth-since ...]

truth.yaml (log_dir) is written by the `truth` command of teach_poses: the TCP position
with the gripper guided by hand flat onto the table, iris centred over the nut. The
k-th truth point after --since is paired with the k-th pick record (also --locate-only)
that has a nut position. Error = truth - computed; its mean is the constant correction
(THEORY 4), its spread what a constant correction cannot remove.
"""
import argparse
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

from sa_tasks import config

REPO_ROOT = config.PACKAGE_DIR.parents[2]
TRUTH_FILE = "truth.yaml"


def truth_path(cfg: dict) -> Path:
    return REPO_ROOT / cfg["log_dir"] / TRUTH_FILE


def append_truth(cfg: dict, tcp_mm, label: str) -> Path:
    path = truth_path(cfg)
    path.parent.mkdir(parents=True, exist_ok=True)
    entries = yaml.safe_load(path.read_text()) if path.is_file() else None
    entries = entries or []
    entries.append({"time": datetime.now().isoformat(timespec="seconds"), "label": label,
                    "tcp_mm": [round(float(v), 2) for v in tcp_mm]})
    path.write_text(yaml.safe_dump(entries, sort_keys=False))
    return path


def correction(truth_xy: np.ndarray, computed_xy: np.ndarray) -> dict:
    """Mean and spread of truth - computed (pure, tested)."""
    errors = np.asarray(truth_xy, float) - np.asarray(computed_xy, float)
    return {"errors": errors, "mean": errors.mean(axis=0),
            "std": errors.std(axis=0, ddof=1) if len(errors) > 1 else np.zeros(2),
            "norm_after": np.linalg.norm(errors - errors.mean(axis=0), axis=1)}


def load_records(directory: Path, since: Optional[str]) -> List[dict]:
    records = []
    for path in sorted(directory.glob("*_pick.yaml")):
        record = yaml.safe_load(path.read_text())
        if record.get("nut") and (since is None or record["started"] >= since):
            record["file"] = path.name
            records.append(record)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--since", help="pick records from this ISO time on, e.g. "
                        "2026-10-18T11:00 (default: all)")
    parser.add_argument("--truth-since", help="truth points from this ISO time on (default: all)")
    parser.add_argument("--config", type=Path, default=config.DEFAULT_TASKS)
    args = parser.parse_args()
    cfg = config.load_tasks(args.config)
    truths = yaml.safe_load(truth_path(cfg).read_text()) if truth_path(cfg).is_file() else []
    truths = [t for t in truths or [] if args.truth_since is None or t["time"] >= args.truth_since]
    records = load_records(REPO_ROOT / cfg["log_dir"], args.since)
    pairs = min(len(truths), len(records))
    if pairs == 0:
        print(f"nothing to compare: {len(truths)} truth points, {len(records)} pick records")
        return
    if len(truths) != len(records):
        print(f"warning: {len(truths)} truth points but {len(records)} pick records, "
              f"comparing the first {pairs} in order")
    truth_xy = np.array([t["tcp_mm"][:2] for t in truths[:pairs]])
    computed_xy = np.array([r["nut"]["position_mm"][:2] for r in records[:pairs]])
    result = correction(truth_xy, computed_xy)
    landed = [(r, t) for r, t in zip(records[:pairs], truth_xy) if r.get("landed_tcp_mm")]
    current = np.asarray(cfg["frames"]["target_correction_mm"], float)
    print(f"{'run':24s} {'computed x/y':>17s} {'truth x/y':>17s} {'error x':>8s} {'error y':>8s}"
          f" {'from axis':>9s}")
    for r, t, e in zip(records[:pairs], truth_xy, result["errors"]):
        c = r["nut"]["position_mm"]
        print(f"{r['file'][:24]:24s} {c[0]:8.1f} {c[1]:8.1f} {t[0]:8.1f} {t[1]:8.1f} "
              f"{e[0]:+8.1f} {e[1]:+8.1f} {r['nut']['offset_from_axis_mm']:7.0f} mm")
    print(f"\nmean error x {result['mean'][0]:+.1f} y {result['mean'][1]:+.1f} mm, "
          f"spread (std) x {result['std'][0]:.1f} y {result['std'][1]:.1f} mm")
    print(f"remaining after a constant correction: max {result['norm_after'].max():.1f} mm")
    print(f"target_correction_mm now {current.round(1).tolist()} -> suggested "
          f"{(current + result['mean']).round(1).tolist()}")
    if landed:
        # Execution: where the robot put the TCP vs. the commanded target (robot error);
        # total: truth vs. where the TCP landed (what is seen at the iris)
        print(f"\n{'run (with descent)':24s} {'landed x/y':>17s} {'exec x':>7s} {'exec y':>7s}"
              f" {'total x':>8s} {'total y':>8s} {'above table':>11s}")
        execution, total = [], []
        for r, t in landed:
            land = np.array(r["landed_tcp_mm"][:2])
            target = np.array(r["tcp_target_mm"][:2])
            execution.append(land - target)
            total.append(t - land)
            print(f"{r['file'][:24]:24s} {land[0]:8.1f} {land[1]:8.1f} {execution[-1][0]:+7.1f} "
                  f"{execution[-1][1]:+7.1f} {total[-1][0]:+8.1f} {total[-1][1]:+8.1f} "
                  f"{r['landed_above_table_mm']:9.1f} mm")
        execution, total = np.array(execution), np.array(total)
        print(f"\nexecution (landed - target): mean x {execution[:, 0].mean():+.1f} "
              f"y {execution[:, 1].mean():+.1f} mm; total (truth - landed): mean x "
              f"{total[:, 0].mean():+.1f} y {total[:, 1].mean():+.1f}, max "
              f"{np.linalg.norm(total, axis=1).max():.1f} mm")
        # The landing contains both the computation and the robot's steady-state error, so
        # the total error is the correction to apply
        print(f"target_correction_mm now {current.round(1).tolist()} -> suggested from the "
              f"landings {(current + total.mean(axis=0)).round(1).tolist()} (use this one)")


if __name__ == "__main__":
    main()
