"""Evaluation of recorded motion (motion_test). Pure NumPy, no ROS. docs/THEORY.md, Section 5."""
import numpy as np


def vibration_rms_mm(t_s: np.ndarray, positions_mm: np.ndarray, window_s: float) -> list:
    """RMS per axis of what a centred moving average over window_s does not explain: the
    fast part of the motion (jitter), not the smooth move itself. positions_mm: (n, 3).
    Fewer samples than one window: [0, 0, 0]."""
    t_s, positions_mm = np.asarray(t_s, float), np.asarray(positions_mm, float)
    if len(t_s) < 3:
        return [0.0, 0.0, 0.0]
    n = int(round(window_s / float(np.median(np.diff(t_s))))) | 1  # odd
    if n < 3 or len(t_s) < n:
        return [0.0, 0.0, 0.0]
    kernel = np.ones(n) / n
    half = n // 2
    rms = []
    for axis in range(positions_mm.shape[1]):
        smooth = np.convolve(positions_mm[:, axis], kernel, mode="valid")
        residual = positions_mm[half:len(positions_mm) - half, axis] - smooth
        rms.append(round(float(np.sqrt(np.mean(residual ** 2))), 4))
    return rms
