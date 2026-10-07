# CLAUDE.md

Semester thesis (ETH): FR3 + eye-in-hand USB camera + 3D-printed iris gripper picks an M6 nut, places it on a fixed stud and screws it on.
**Full context, design decisions and open questions: `docs/PLAN.md` – read it before larger changes and update it (decision log) when a design choice changes.**
Deadline 31.10.2026 – prefer the simplest thing that raises the success rate.

## Conventions
- Code, comments, logs, commit messages: English. Planning docs (`docs/PLAN.md`): German.
- ROS2 workspace in `software/` (rclpy, ament_python; interfaces in ament_cmake). Must work on Humble and Jazzy.
- Pure logic (detection, geometry, serial protocol, state transitions) must not import rclpy → unit-testable with pytest on any laptop.
- No magic numbers in code: everything tunable lives in `software/src/sa_tasks/config/*.yaml`.
- Hardware behind interfaces with mock implementations (robot, gripper, camera).
- Gripper is purely time-based (no sensors). Timing runs on the Arduino; PC sends durations.
- Firmware: `arduino/iris_gripper/` (Arduino UNO R4 Minima, Adafruit Motor Shield v2.3). Port mapping only via constants at top of the sketch.

## Hardware quick facts
- Iris motors: M2 + M4 (opposite directions), rotation: M1 (slip ring, unlimited). Provisional.
- Camera offset to gripper centre: dx 0, dy −44.505 mm, dz 52.1 mm (CAD, frame see PLAN F21).
- Camera: HutoPi 720p (OV9726), looks straight down.
