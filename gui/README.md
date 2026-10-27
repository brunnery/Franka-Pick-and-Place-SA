# SA launcher (temporary)

> **Temporary lab helper, not part of the thesis code. Delete this folder before the
> final submission** (`git rm -r gui`, plus the pointer lines in `README.md` and
> `docs/STATUS.md`). Nothing outside `gui/` imports it, and it changes no thesis code.

First thing tomorrow: [NEXT_SESSION.md](NEXT_SESSION.md).

One window instead of copy-pasting commands:

- **Stud side**: click the other area to swap. The GUI exchanges the x/y of `observe_nut` and
  `observe_stud` in `software/src/sa_tasks/config/poses.yaml` (z and orientation stay, so the
  shared observation height does not change). That is the same swap as by hand; every later
  run uses it, also from a plain terminal. Left = +y seen from the robot base (= left in the
  camera image).
- **Buttons**: each one opens its own terminal (`gnome-terminal`, otherwise `xterm`) with
  ROS, `~/franka_ros2_ws` and `software/install` sourced, starting in the repository root.
  Grey = idle, orange = starting, green = running. ■ (or "Stop all") sends Ctrl-C; the
  terminal stays open with the log, Enter closes it. Robot moves ask the safety question
  first. Starting something that opens the robot, camera or Arduino while another running
  button already has it asks first (the run launches start robot + vision + gripper
  themselves). Hover over a button to see its command.
- **Run timer** (top left, needs ROS): live time of the current run, plus when the iris closed
  (pick) and when the M4 rotation stopped (end of tightening). t = 0 when robot, gripper and
  vision are up; the clock stops when the iris starts to open at the stud (cycle time). Comes from the time marks sa_tasks publishes on `/sa_tasks/events`.
- **Rate the last run**: screwed on / partly / failed, turns, failed phase, note → "Save
  rating" appends `outcome:` to the newest `data/runs/*_full.yaml` (rating again replaces
  it). `run_map` (button "Run map + summary") uses it for the map of the runs. On top: the
  run counter of the current test round (`run_record.round` in `sa_tasks/config/tasks.yaml`),
  e.g. "round 2: run 7 / 20" and how many were screwed on, partly, failed, aborted, not rated.
- **Images**: the two halves of `/vision/debug` on the right, binary mask and result with the
  fitted polygon (buttons use `view:=false`, so no extra rqt windows). The raw camera image
  only via the rqt button.

## Start

```bash
ln -s ~/<repo>/gui/sa_gui ~/.local/bin/sa_gui   # once
sa_gui                                          # from a normal terminal, not VS Code's
```

Buttons, ports and paths: `gui.yaml` (add a button = add an entry). The upload buttons need
`arduino-cli` with the `arduino:renesas_uno` core and the *Adafruit Motor Shield V2 Library*;
without it, upload with the Arduino IDE.

Logic tests (no ROS, no Qt): `cd gui && python3 -m pytest test_launcher.py`
