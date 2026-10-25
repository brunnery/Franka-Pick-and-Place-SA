import numpy as np

from sa_tasks.analysis import vibration_rms_mm


def test_smooth_motion_has_no_vibration():
    t = np.arange(0.0, 2.0, 0.001)
    path = np.stack([100.0 * t, 0.0 * t, -50.0 * t ** 2], axis=1)
    # A moving average over w is off by a * w^2 / 24 on a parabola: 100 mm/s^2 -> 0.01 mm
    assert max(vibration_rms_mm(t, path, 0.05)) < 0.02


def test_fast_oscillation_is_measured():
    t = np.arange(0.0, 2.0, 0.001)
    path = np.zeros((len(t), 3))
    path[:, 2] = 100.0 * t + 0.5 * np.sin(2.0 * np.pi * 80.0 * t)  # 80 Hz, 0.5 mm amplitude
    rms = vibration_rms_mm(t, path, 0.05)
    assert rms[0] == 0.0
    assert abs(rms[2] - 0.5 / np.sqrt(2.0)) < 0.05


def test_too_few_samples():
    assert vibration_rms_mm([0.0, 0.001], np.zeros((2, 3)), 0.05) == [0.0, 0.0, 0.0]


def test_uneven_sampling_is_not_jitter():
    rng = np.random.default_rng(0)
    t = np.sort(rng.uniform(0.0, 2.0, 1000))
    path = np.stack([250.0 * t, 0.0 * t, 0.0 * t], axis=1)  # 250 mm/s, unevenly sampled
    assert max(vibration_rms_mm(t, path, 0.05)) < 1e-3
