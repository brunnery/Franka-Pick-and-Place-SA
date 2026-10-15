"""The shipped YAML files must match the code (missing entries fail loudly)."""
from pathlib import Path

import pytest

from sa_vision.camera_controls import load_controls, readback_command, v4l2_commands
from sa_vision.params import D_SOURCES, distance_range, load_vision_config

PACKAGE = Path(__file__).resolve().parents[1]
CAMERA_CONFIG = PACKAGE.parents[1] / "config" / "camera.yaml"


def test_vision_yaml_provides_all_nut_parameters():
    config = load_vision_config(PACKAGE / "config" / "vision.yaml")
    assert config.nut.target_across_flats_mm == 13.0  # M8 locknut
    assert config.d_source in D_SOURCES
    assert len(config.d_from_size_range_mm) == 2
    assert len(config.nut.across_flats_range_mm) == 2
    assert config.reference_image


def test_camera_yaml_switches_automatic_modes_off_before_manual_values():
    device, controls = load_controls(CAMERA_CONFIG)
    names = list(controls)
    assert device.startswith("/dev/video")
    assert controls["auto_exposure"] == 1 and controls["white_balance_automatic"] == 0
    assert names.index("auto_exposure") < names.index("exposure_time_absolute")
    assert names.index("white_balance_automatic") < names.index("white_balance_temperature")


def test_v4l2_commands_keep_order():
    controls = {"auto_exposure": 1, "exposure_time_absolute": 500}
    assert v4l2_commands("/dev/video0", controls) == [
        ["v4l2-ctl", "-d", "/dev/video0", "-c", "auto_exposure=1"],
        ["v4l2-ctl", "-d", "/dev/video0", "-c", "exposure_time_absolute=500"],
    ]
    assert readback_command("/dev/video0", controls) == [
        "v4l2-ctl", "-d", "/dev/video0", "-C", "auto_exposure,exposure_time_absolute"]


def test_distance_range_depends_on_d_source():
    assert distance_range("yaml", [200.0, 500.0]) is None
    assert distance_range("from_size", [200.0, 500.0]) == (200.0, 500.0)
    with pytest.raises(ValueError):
        distance_range("fk", [200.0, 500.0])
