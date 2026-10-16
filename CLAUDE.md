# CLAUDE.md

Semester thesis (ETH): FR3 + eye-in-hand USB camera + 3D-printed iris gripper picks a hex nut (test part: M8; method should work for common metric sizes), places it on a fixed stud and screws it on.
**Full context, design decisions and open questions: `docs/PLAN.md` – read it before larger changes and update it (decision log) when a design choice changes.**
Deadline 31.10.2026 – prefer the simplest thing that raises the success rate.
**Theory for the report: `docs/THEORY.md`.** Whenever a formula, model or principle is decided or used in code, add or update it there (English, report-ready, with status used/planned/dropped) and commit it together with the code change.

## Conventions
- **The bachelor thesis (BA) is not a source.** Do not take code, parameters, frames, offsets or methods from it; design from the current hardware and this repo. BA numbers appear only as a comparison in the report.
- Code, comments, logs, commit messages: English. Planning docs (`docs/PLAN.md`): German.
- ROS2 workspace in `software/` (rclpy, ament_python; interfaces in ament_cmake). Must work on Humble and Jazzy.
- Pure logic (detection, geometry, serial protocol, state transitions) must not import rclpy → unit-testable with pytest on any laptop.
- No magic numbers in code: everything tunable lives in the YAML of its package, `software/src/<pkg>/config/*.yaml`. Nodes declare parameters without defaults, so a missing YAML entry fails loudly.
- Environment: ROS 2 Humble, system NumPy 1.21 / OpenCV 4.5. Always run with `PYTHONNOUSERSITE=1` (NumPy 2 in `~/.local` breaks ROS OpenCV); never pip install. Build: `cd software && colcon build --symlink-install`. Unit tests: `cd software/src/<pkg> && python3 -m pytest test`.
- Hardware behind interfaces with mock implementations (robot, gripper, camera).
- Gripper is purely time-based (no sensors). Timing runs on the Arduino; PC sends durations.
- Firmware: `arduino/iris_gripper/` (Arduino UNO R4 Minima, Adafruit Motor Shield v2.3). Port mapping only via constants at top of the sketch.

## Hardware quick facts
- Iris motors: M2 + M4 (opposite directions), rotation: M1 (slip ring, unlimited). Provisional.
- Flange centre → camera centre, lowest point of the camera housing (lens 5 mm higher): dx 0, dy +44.4 mm, dz 57.223 mm in the flange frame (CAD 16.10.; Fusion in base axes: (44.4, 0, −57.223)). Camera always sits in front of the flange (+x_robo): fixed EE orientation x_EE = +y_robo, y_EE = +x_robo, z_EE = −z_robo.
- Axis mapping (gripper pointing down, viewed from base): x_robo = −y_cam, y_robo = −x_cam, z_robo = −z_cam (OpenCV camera frame: x right, y down in image, z into scene).
- Flange centre → gripper bottom-plate centre (TCP, rests on table): (0, 0, 118.823) mm (CAD 16.10.). Flange centre = centre of the FR3 flange contact face (the adapter's centring cylinder reaches into the arm). All offsets in mm, CAD values < 0.2 mm treated as 0.
- Camera: HutoPi 720p (OV9726), looks straight down.
