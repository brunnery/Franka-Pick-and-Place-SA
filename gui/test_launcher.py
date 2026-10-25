"""Tests of the launcher logic: cd gui && python3 -m pytest test_launcher.py"""
from pathlib import Path

import pytest
import yaml

import launcher

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
CFG = launcher.load_config(HERE / "gui.yaml")
POSES_TEXT = (REPO / CFG["poses_file"]).read_text(encoding="utf-8")


def entry(label):
    for _, entries in launcher.groups(CFG):
        for e in entries:
            if e.label == label:
                return e
    raise KeyError(label)


def all_entries():
    return [e for _, entries in launcher.groups(CFG) for e in entries]


def test_every_button_formats_and_keys_are_unique():
    keys = [e.key for e in all_entries()]
    assert len(keys) == len(set(keys)) and keys
    assert all("{" not in e.command or "$(" in e.command for e in all_entries())
    assert "/dev/serial/by-id/" in entry("Upload iris_gripper firmware").command


def test_arduino_port_matches_the_gripper_port_of_tasks_yaml():
    tasks = yaml.safe_load((REPO / "software/src/sa_tasks/config/tasks.yaml")
                           .read_text(encoding="utf-8"))
    assert CFG["variables"]["arduino_port"] == tasks["gripper"]["port"]


def test_robot_moves_ask_for_confirmation():
    assert entry("Full run (sa_full_all)").confirm
    assert entry("Jitter test (motion_test)").confirm
    assert not entry("Build sa packages").confirm


def test_buttons_call_existing_launch_files_sketches_and_entry_points():
    setup = (REPO / "software/src/sa_tasks/setup.py").read_text(encoding="utf-8")
    for e in all_entries():
        words = e.command.split()
        for word in words:
            if word.startswith(("software/launch/", "arduino/")):
                assert (REPO / word).exists(), f"{e.label}: {word} missing"
        if words[:3] == ["ros2", "run", "sa_tasks"]:
            assert f'"{words[3]} = ' in setup, f"{e.label}: no entry point {words[3]}"


def test_starting_the_same_hardware_twice_is_reported():
    full = entry("Full run (sa_full_all)")
    vision = entry("Vision nut")
    arduino = entry("Arduino test (sa_arduino_test)")
    assert launcher.conflicts(vision, [full]) == [(full.label, "camera")]
    assert launcher.conflicts(arduino, [vision]) == []
    assert launcher.conflicts(full, [full]) == []  # itself does not count


def test_shell_script_sources_env_and_writes_the_pid(tmp_path):
    pid = tmp_path / "x.pid"
    script = launcher.shell_script(entry("Pick only (sa_pick_all)"), CFG, REPO, pid)
    assert "export PYTHONNOUSERSITE=1" in script
    assert "opt/ros/humble/setup.bash" in script.replace("\\", "/")
    assert f"source {REPO / 'software' / 'install' / 'setup.bash'}" in script.replace("'", "")
    assert script.index("echo $$") < script.index("pick.launch.py view:=false\n")
    assert "trap ':' INT" in script


def test_terminal_falls_back_to_the_first_installed():
    cmd = launcher.terminal_command(CFG, "Pick", Path("/tmp/p.sh"),
                                    which=lambda name: name == "xterm")
    assert cmd == ["xterm", "-T", "Pick", "-e", "bash", str(Path("/tmp/p.sh"))]
    with pytest.raises(RuntimeError, match="no terminal"):
        launcher.terminal_command(CFG, "Pick", Path("/tmp/p.sh"), which=lambda name: None)


def test_running_pid_removes_stale_files(tmp_path):
    pid = tmp_path / "x.pid"
    assert launcher.running_pid(pid) is None
    pid.write_text("4242\n")
    assert launcher.running_pid(pid, alive=lambda p: True) == 4242
    assert launcher.running_pid(pid, alive=lambda p: False) is None
    assert not pid.exists()


def test_swapping_the_areas_exchanges_xy_only_and_keeps_the_header():
    before = yaml.safe_load(POSES_TEXT)
    swapped_text = launcher.swap_areas(POSES_TEXT)
    after = yaml.safe_load(swapped_text)
    nut, stud = (before["poses"][n] for n in (launcher.NUT_POSE, launcher.STUD_POSE))
    assert after["poses"][launcher.NUT_POSE]["position_mm"] == (
        stud["position_mm"][:2] + nut["position_mm"][2:])
    assert after["poses"][launcher.STUD_POSE]["position_mm"] == (
        nut["position_mm"][:2] + stud["position_mm"][2:])
    assert after["poses"][launcher.NUT_POSE]["quaternion_xyzw"] == nut["quaternion_xyzw"]
    assert after["table_z_mm"] == before["table_z_mm"]
    assert swapped_text.startswith("# Taught poses")
    assert launcher.stud_side(after) != launcher.stud_side(before)
    assert yaml.safe_load(launcher.swap_areas(swapped_text)) == before  # swap twice = back


def test_stud_side_is_where_observe_stud_lies():
    poses = {"poses": {launcher.NUT_POSE: {"position_mm": [340.0, 222.0, 463.0]},
                       launcher.STUD_POSE: {"position_mm": [410.0, -233.0, 473.0]}}}
    assert launcher.stud_side(poses) == "right"
    assert launcher.area_xy(poses)[launcher.STUD_POSE] == [410.0, -233.0]
