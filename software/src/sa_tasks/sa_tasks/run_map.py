"""Evaluation of all run records: one CSV row per run and a map of where nut and stud were.

  ros2 run sa_tasks run_map [--since 20261027] [--round "round 5" "round 6"] [--steps full ...]
  ros2 run sa_tasks run_map --times 5     # only print the time per step of the last 5 runs

Reads <log_dir>/<YYYYMMDD-HHMMSS>_<step>.yaml (tasks.yaml run_map: since, steps) and writes
the CSV and the PNG of run_map next to them. Left plot: computed nut (blue hexagon) and stud
(red square) positions in the base frame seen from the robot base (+x up, +y left), fill =
outcome, the run number beside both; dashed boxes = what the camera sees from the observation
poses. Right plot: TCP minus computed target at the pick landing and at the end of screwing. The outcome comes from the
rating in the GUI (record.outcome), else from the result. No ROS.
"""
import argparse
import csv
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

from sa_tasks import config, sequences

SHAPES = {"nut": "h", "stud": "s"}  # matplotlib markers: hexagon, square (the stud's base)
# outcome -> (fill style, marker drawn on top or None, legend text)
MARKERS = {"success": ("full", None, "screwed on"),
           "partly": ("left", None, "partly"),
           "failed": ("none", "x", "failed"),
           "aborted": ("none", "+", "aborted / error"),
           "unrated": ("none", None, "not rated")}
SCREWED = {"yes": "success", "partly": "partly", "no": "failed"}


def status(record: dict) -> str:
    """success / partly / failed (rated in the GUI), else aborted (no result 'ok') or unrated."""
    rating = SCREWED.get((record.get("outcome") or {}).get("screwed"))
    if rating:
        return rating
    return "unrated" if record.get("result") == "ok" else "aborted"


def step_times(timeline: dict) -> dict:
    """{mark: seconds since the mark before} from the cumulative timeline_s of a record."""
    times, previous = {}, 0.0
    for name, t in timeline.items():
        times[name] = round(float(t) - previous, 2)
        previous = float(t)
    return times


def times_table(rows: List[dict]) -> str:
    """Time per step (rows) of each run (columns), plus the total."""
    columns = [(row["record"][9:15], step_times(row["timeline"])) for row in rows]
    steps = list(dict.fromkeys(name for _, times in reversed(columns) for name in times))
    lines = ["step".ljust(20) + "".join(label.rjust(8) for label, _ in columns)]
    for name in steps:
        lines.append(name.ljust(20) + "".join(
            ("" if name not in times else f"{times[name]:.1f}").rjust(8) for _, times in columns))
    lines.append("total".ljust(20) + "".join(f"{sum(times.values()):.1f}".rjust(8)
                                               for _, times in columns))
    return "\n".join(lines)


def _xy(value) -> Optional[np.ndarray]:
    return None if not value else np.asarray(value[:2], float)


def _minus(a, b) -> list:
    """a - b in xy, [None, None] if either is missing."""
    a, b = _xy(a), _xy(b)
    return [None, None] if a is None or b is None else [round(float(v), 2) for v in a - b]


def summarise(name: str, record: dict) -> dict:
    """One flat row per run. Errors are computed from the raw positions, so older records
    (without the *_minus_target keys of 27.10.) fit in too."""
    pick, place = record.get("pick") or {}, record.get("place") or {}
    stud = place.get("stud") or {}
    screw = place.get("screw") or {}
    search = place.get("thread_search") or {}
    outcome = record.get("outcome") or {}
    timeline = record.get("timeline_s") or {}
    nut_xy, stud_xy = _xy(pick.get("nut_mm")), _xy(stud.get("position_mm"))
    row = {"record": name, "round": record.get("round"), "step": record.get("step"),
           "started": record.get("started"),
           "status": status(record), "result": record.get("result"),
           "turns": outcome.get("turns"), "failed_phase": outcome.get("failed_phase"),
           "note": outcome.get("note"),
           "duration_s": record.get("duration_s"), "setup_s": record.get("setup_s"),
           "t_iris_closed_s": timeline.get("iris_closed"),
           "t_m4_stopped_s": timeline.get("m4_stopped"),
           "commit": (record.get("config") or {}).get("commit")}
    for label, xy in (("nut", nut_xy), ("stud", stud_xy)):
        row[f"{label}_x"], row[f"{label}_y"] = [None, None] if xy is None else xy.round(2)
        row[f"{label}_side"] = None if xy is None else ("left" if xy[1] > 0 else "right")
    pairs = {"nut_close": (pick.get("nut_mm"), pick.get("nut_coarse_mm")),
             "landed": (pick.get("landed_tcp_mm"), pick.get("nut_mm")),
             "stud_close": (stud.get("close_mm"), stud.get("observed_mm")),
             "approach": (place.get("approach_reached_tcp_mm"), stud.get("position_mm")),
             "contact": (place.get("contact_tcp_mm"), stud.get("position_mm")),
             "thread": (search.get("lowest_tcp_mm"), stud.get("position_mm")),
             "end": (screw.get("end_tcp_mm"), stud.get("position_mm"))}
    for label, (a, b) in pairs.items():
        row[f"{label}_dx"], row[f"{label}_dy"] = _minus(a, b)
    row.update({f"dt_{name}_s": dt for name, dt in step_times(timeline).items()})
    row.update(pick_contact=pick.get("contact"), place_contact=place.get("contact"),
               seat_travel_mm=(place.get("seat") or {}).get("travel_mm"),
               thread_drop_mm=search.get("drop_mm"), caught=screw.get("caught"),
               drop_end_mm=screw.get("drop_end_mm"))
    return row


def load(directory: Path, since: str, steps: List[str], rounds: List[str] = ()) -> List[dict]:
    """Rows of the records <timestamp>_<step>.yaml from since on (file name order), only those
    of the given rounds unless there are none."""
    rows = []
    for path in sorted(Path(directory).glob("*-*_*.yaml")):
        if path.name < since:
            continue
        try:
            record = yaml.safe_load(path.read_text())
        except yaml.YAMLError:
            continue
        if (isinstance(record, dict) and record.get("step") in steps
                and (not rounds or record.get("round") in rounds)):
            rows.append({"run": len(rows) + 1, **summarise(path.stem, record),
                         "timeline": record.get("timeline_s") or {}})
    return rows


def view_areas(cfg: dict, poses: config.Poses, half_size_mm) -> List[tuple]:
    """(centre xy, half size xy) on the table of what the camera sees from each observation
    pose: centred where the optical axis meets the table."""
    table_z = poses.table_z()
    areas = []
    for name in ("observe_nut", "observe_stud"):
        flange = sequences.observation_pose(poses, name, cfg)
        camera = flange @ config.camera_in_flange(cfg, flange[1, 3])
        axis = camera[:3, 2]  # z_C in the base frame, points down
        centre = camera[:3, 3] + (table_z - camera[2, 3]) / axis[2] * axis
        areas.append((centre[:2], np.asarray(half_size_mm, float)))
    return areas


def write_csv(rows: List[dict], path: Path) -> None:
    columns = list(dict.fromkeys(key for row in rows for key in row if key != "timeline"))
    with open(path, "w", newline="") as file:
        writer = csv.DictWriter(file, columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def plot(rows: List[dict], colours: dict, path: Path, areas=()) -> None:
    import matplotlib
    matplotlib.use("Agg")  # file only, no display needed
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    figure, (positions, errors) = plt.subplots(1, 2, figsize=(14, 7))

    def draw(ax, x, y, obj, state, number):
        fill, overlay, _ = MARKERS[state]
        ax.plot(x, y, linestyle="none", marker=SHAPES[obj], fillstyle=fill, markersize=11,
                color=colours[obj], markeredgecolor=colours[obj], alpha=0.85)
        if overlay:
            ax.plot(x, y, linestyle="none", marker=overlay, markersize=9, color="#222222")
        ax.annotate(str(number), (x, y), textcoords="offset points", xytext=(6, 5),
                    fontsize=7, color="#444444")

    # plot axes: y_B horizontal (to the left), x_B up
    for (cx, cy), (hx, hy) in areas:
        positions.add_patch(plt.Rectangle((cy - hy, cx - hx), 2 * hy, 2 * hx, fill=False,
                                          color="#999999", lw=1.0, linestyle="--"))
        positions.text(cy, cx + hx, f"{'left' if cy > 0 else 'right'} area", ha="center",
                       va="bottom", fontsize=8, color="#777777")
    for row in rows:
        for obj in ("nut", "stud"):
            if row[f"{obj}_x"] is not None:
                draw(positions, row[f"{obj}_y"], row[f"{obj}_x"], obj, row["status"], row["run"])
        for obj, label in (("nut", "landed"), ("stud", "end")):
            if row[f"{label}_dx"] is not None:
                draw(errors, row[f"{label}_dy"], row[f"{label}_dx"], obj, row["status"],
                     row["run"])

    rounds = sorted({str(row["round"]) for row in rows if row.get("round")})
    positions.set_title(f"Computed positions, {len(rows)} runs {', '.join(rounds)}\n"
                        "(seen from the robot base)")
    positions.set_xlabel("y_B [mm]  (+ = left)")
    positions.set_ylabel("x_B [mm]")
    errors.set_title("TCP minus computed target\nnut: pick landing, stud: end of screwing")
    errors.set_xlabel("Δy_B [mm]  (+ = left)")
    errors.set_ylabel("Δx_B [mm]")
    for radius in (1.0, 2.0, 4.0):
        errors.add_patch(plt.Circle((0, 0), radius, fill=False, color="#bbbbbb", lw=0.8))
    errors.axhline(0, color="#dddddd", lw=0.8)
    errors.axvline(0, color="#dddddd", lw=0.8)
    for ax in (positions, errors):
        ax.invert_xaxis()  # +y to the left, as seen from the base
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, color="#eeeeee")

    counts = {state: sum(row["status"] == state for row in rows) for state in MARKERS}
    handles = [Line2D([], [], linestyle="none", marker=SHAPES[obj], color=colours[obj],
                      markersize=9, label=obj) for obj in ("nut", "stud")]
    for state, (fill, overlay, text) in MARKERS.items():
        if counts[state]:
            handles.append(Line2D([], [], linestyle="none", marker=overlay or "o",
                                  fillstyle=fill, color="#444444", markersize=8,
                                  label=f"{text} ({counts[state]})"))
    positions.legend(handles=handles, loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main():
    cfg = config.load_tasks()
    settings = cfg["run_map"]
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--since", default=settings["since"],
                        help="first record, file name prefix (e.g. 20261027-1400); '' = all")
    parser.add_argument("--steps", nargs="+", default=settings["steps"])
    default_rounds = settings["round"]  # one round (str) or several (list); "" / [] = all
    if isinstance(default_rounds, str):
        default_rounds = [default_rounds] if default_rounds else []
    parser.add_argument("--round", nargs="*", default=default_rounds,
                        help="only records of these run_record.round values; none = all")
    parser.add_argument("--times", type=int, metavar="N",
                        help="only print the time per step of the last N runs (any date)")
    args = parser.parse_args()
    directory = config.PACKAGE_DIR.parents[2] / cfg["log_dir"]
    if args.times:
        rows = [row for row in load(directory, "", args.steps) if row["timeline"]]
        print(times_table(rows[-args.times:]) if rows else "no records with time marks")
        return
    rows = load(directory, args.since, args.steps, args.round)
    if not rows:
        print(f"no {'/'.join(args.steps)} records from {args.since or 'the start'} on "
              f"{'of ' + ', '.join(args.round) + ' ' if args.round else ''}in {directory}")
        return
    write_csv(rows, directory / settings["csv"])
    plot(rows, settings["colours"], directory / settings["png"],
         view_areas(cfg, config.Poses(), settings["area_half_size_mm"]))
    counts = {state: sum(row["status"] == state for row in rows) for state in MARKERS}
    print(f"{len(rows)} runs: " + ", ".join(f"{state} {n}" for state, n in counts.items() if n))
    print(f"written: {directory / settings['csv']}\n         {directory / settings['png']}")
