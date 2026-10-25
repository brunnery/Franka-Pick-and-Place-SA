# Launcher GUI – summary of 25.10.2026 and checklist for the workstation

Temporary note; lives in `gui/` and goes away with it (`git rm -r gui`).

## What it is

A window (`gui/sa_gui`) with:

- **Stud side switch** (top view seen from the robot base, left = +y): click the other area and
  the x/y of `observe_nut` and `observe_stud` in `poses.yaml` are swapped (z stays, so the
  shared observation height is unchanged). Same as swapping by hand; terminal runs use it too.
  The areas show the current coordinates. State today: **stud right (410, −233), nut left
  (340, +222)**.
- **One button per command**, each in its own terminal, status light, ■ = Ctrl-C. Robot moves
  ask the safety question first. Conflict warning if robot/camera/Arduino is already in use
  by another running button (the run launches start all three themselves).
- **Camera and vision debug image** embedded on the right.

Buttons (all from the current code / STATUS.md aliases):

| Group | Buttons |
|---|---|
| Runs | Full run (`sa_full_all`), Pick only (`sa_pick_all`), Place + screw, Bolt approach only (`sa_place_approach`) |
| Robot | Robot / controller (`sa_robot`, `record:=false`), Robot state only, Teach poses |
| Robot tests (need `sa_robot`) | Go to observation poses (`goto_poses`), Jitter test (`motion_test`), Reach test, Hand-eye calibration |
| Vision | Vision nut, Vision bolt (`sa_vision_bolt`), Camera only |
| Gripper / Arduino | Arduino test (`gripper_test` = `sa_arduino_test`), Screw test, upload iris_gripper / test M2-M3 / test M4 (arduino-cli) |
| Last run / checks | screwing of last run, pick landing, lost control cycles, unit tests, build |

**No thesis code was changed.** Only `gui/` is new, plus pointer lines in `README.md`,
`docs/STATUS.md` and a PLAN.md log entry.

Tested on the laptop: 10 unit tests of the GUI logic; window rendered; side swap twice gives a
byte-identical `poses.yaml`; terminal scripts are valid bash and clean up their pid file.
**Not tested: anything with ROS, robot, terminals on Ubuntu.**

## Checklist at the workstation

1. [ ] `git pull`
2. [ ] Once: `ln -s ~/<repo>/gui/sa_gui ~/.local/bin/sa_gui`
3. [ ] Start `sa_gui` from a **normal terminal** (not VS Code: snap issue)
4. [ ] Harmless first: "Unit tests" → does a terminal open, is ROS sourced?
5. [ ] "Vision nut" → do both image panes show pictures? Stop it again with ■.
6. [ ] Then use it as usual, e.g. "Full run". The confirm dialog replaces nothing: hand on the
       emergency stop.
7. [ ] Stud on the other side? Click it in the window, check the coordinates.
       **Note:** `frames.target_correction` (linear in y, support points y −300 / −130) was
       tuned with the stud on the right; with the stud on the left (y ≈ +222) it uses the end
       value [0, −1] mm, untested there.
8. [ ] Upload buttons only work with `arduino-cli`; otherwise the Arduino IDE as before.

If something is wrong: buttons and paths are in `gui/gui.yaml`.

## Housekeeping

- This laptop had been 62 commits behind (state of 19.10.); the first GUI draft built on that
  old state is parked in the **local** branch `wip/gui-on-old-base` (not pushed). It can be
  deleted: `git branch -D wip/gui-on-old-base`.
- Before submission: `git rm -r gui` and remove the pointer lines (README.md, STATUS.md).
