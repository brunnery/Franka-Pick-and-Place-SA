"""Temporal filter of the nut centre: median over the last n frames. Pure Python, no ROS.

docs/THEORY.md, Section 2.7. Rim pixels close to the threshold flip from frame to
frame and move the single-frame centre by about 1 px; the median of n frames removes
this jitter and single-frame outliers. The window starts over after an invalid frame
or when the nut jumps (camera or nut moved), so the median never mixes two positions.
"""
from collections import deque
from dataclasses import dataclass
import math
import statistics
from typing import Optional

from sa_vision.detection import NutDetection


@dataclass(frozen=True)
class FilteredCentre:
    u: float
    v: float
    distance_mm: float
    frames: int   # frames in the median
    window: int   # frames needed for a settled result

    @property
    def settled(self) -> bool:
        return self.frames >= self.window


class CentreMedian:
    def __init__(self, window: int, reset_jump_ratio: float):
        """window: n frames; reset_jump_ratio: jump / s_px that starts the window over."""
        if window < 1:
            raise ValueError("the median window needs at least one frame")
        self.window = window
        self.reset_jump_ratio = reset_jump_ratio
        self.samples = deque(maxlen=window)  # (u, v, d)

    def clear(self) -> None:
        self.samples.clear()

    def update(self, detection: NutDetection) -> Optional[FilteredCentre]:
        """Add one frame; None if the frame has no valid nut (the window starts over)."""
        if not detection.valid:
            self.clear()
            return None
        if self.samples:
            current = self._median()
            jump = math.hypot(detection.u - current.u, detection.v - current.v)
            if jump > self.reset_jump_ratio * detection.across_flats_px:
                self.clear()
        self.samples.append((detection.u, detection.v, detection.distance_mm))
        return self._median()

    def _median(self) -> FilteredCentre:
        us, vs, ds = zip(*self.samples)
        return FilteredCentre(statistics.median(us), statistics.median(vs),
                              statistics.median(ds), len(self.samples), self.window)
