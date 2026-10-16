# Attribution

- **Copied from:** `external/cartesian_impedance_control/` in this repo (supervisor repo, branch `main`, commit
  `f8d5fe258afe9f45fc1fe1f3821a4b85a7be8f98`, imported 2026-10-09; see docs/external-repos.md).
- **Original author (per README.md):** Curdin Deplazes (Pd|Z, ETH Zurich),
  https://github.com/CurdinDeplazes/cartesian_impedance_control
- **License:** Apache 2.0 (`LICENSE`, `package.xml`); source headers keep the Franka Emika GmbH copyright.
- **Version:** package.xml 0.1.8. The README targets franka_ros2 v0.13.1; this workspace uses franka_ros2 v2.2.0 +
  libfranka 0.19.0 (container thesis_franka, docs/docker.md). Needed changes: docs/controller-setup.md.
- **Changes vs. the original:** copied identical to `external/cartesian_impedance_control/` (checked with
  `diff -r` on 2026-10-13), plus this file. Changed since:
  - 2026-10-13 (Matteo, first robot test): translational stiffness x/y 2500 → 1000 in `K`
    (`include/cartesian_impedance_control/cartesian_impedance_controller.hpp`); z (1000) and rotation unchanged.
  - 2026-10-13: launch file adapted to franka_ros2 v2.2.0 (`launch/cartesian_impedance_controller.launch.py`):
    `robot_type` (default fr3) instead of `arm_id`, `robot_ip` default 192.168.1.200, `load_gripper` default false,
    `controllers_yaml:=` this package's `launch/controllers.yaml`; removed the `/home/lucas/.../set_force_torque_limits.sh`
    ExecuteProcess + OnProcessExit chain (controller spawned directly, active) and the `use_rviz` argument.
    `launch/controllers.yaml` replaced with the content of `ros2_ws/src/thesis_bringup/config/controllers.yaml`
    (v2.2.0 base: update_rate, thread_priority, both broadcasters, controller type + `arm_id`/`robot_type` fr3).
  - 2026-10-13: tuning parameters in `launch/controllers.yaml` (docs/controller-tuning.md): all gains (K, damping
    ratios, integrator, clamps, filter, torque rate, mode presets) read in `on_configure`, defaults = the old
    hardcoded values; new optional (off by default): nullspace posture spring + damping, Λ regularisation,
    integrator reset on a new target, spring-only clamp. Note: the start-up K default is the .hpp value (x/y 1000).
  - 2026-10-13: `max_reference_velocity` (rate limit of the reference, default off); yaml switched to the first
    tuning set (docs/decisions.md).
  - 2026-10-13: debug topic `~/debug` (realtime_tools RealtimePublisher, `debug.publish_rate`); new dependency
    realtime_tools (package.xml, CMakeLists.txt). yaml back to the baseline values.
  - 2026-10-13: exact nullspace projector (SVD) instead of the damped pinv, fixed posture target
    (`nullspace.q_target`), smooth start move at activation (`start_pose.*`).
  - 2026-10-13: launch file records the debug topic with ros2 bag (`record`, `bag_dir`, `bag_name` arguments).
- **Depends on:** `messages_fr3` (ros2_ws/src/messages_fr3, see its ATTRIBUTION.md).
