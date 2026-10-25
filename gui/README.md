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
- **Images**: `/image_raw` and `/vision/debug` on the right (buttons use `view:=false`, so no
  extra rqt windows).

## Start

```bash
ln -s ~/<repo>/gui/sa_gui ~/.local/bin/sa_gui   # once
sa_gui                                          # from a normal terminal, not VS Code's
```

Buttons, ports and paths: `gui.yaml` (add a button = add an entry). The upload buttons need
`arduino-cli` with the `arduino:renesas_uno` core and the *Adafruit Motor Shield V2 Library*;
without it, upload with the Arduino IDE.

Logic tests (no ROS, no Qt): `cd gui && python3 -m pytest test_launcher.py`
