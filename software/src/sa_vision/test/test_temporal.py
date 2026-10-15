import math

import pytest

from sa_vision.detection import NutDetection
from sa_vision.temporal import CentreMedian

S_PX = 80.0


def nut(u, v, d=300.0):
    return NutDetection(True, "ok", u=u, v=v, distance_mm=d, across_flats_px=S_PX)


def no_nut():
    return NutDetection(False, "no nut found")


def test_median_removes_jitter_and_outlier():
    median = CentreMedian(5, 0.25)
    for u in (100.0, 101.0, 99.0, 100.5, 104.0):  # 104: outlier, below the jump limit
        result = median.update(nut(u, 50.0))
    assert result.settled
    assert (result.u, result.v) == (100.5, 50.0)


def test_settled_only_with_a_full_window():
    median = CentreMedian(3, 0.25)
    assert not median.update(nut(10.0, 10.0)).settled
    assert median.update(nut(10.0, 10.0)).frames == 2
    assert median.update(nut(10.0, 10.0)).settled


def test_invalid_frame_starts_the_window_over():
    median = CentreMedian(3, 0.25)
    median.update(nut(10.0, 10.0))
    median.update(nut(10.0, 10.0))
    assert median.update(no_nut()) is None
    assert median.update(nut(10.0, 10.0)).frames == 1


def test_jump_starts_the_window_over():
    median = CentreMedian(3, 0.25)
    for _ in range(3):
        median.update(nut(100.0, 100.0))
    result = median.update(nut(100.0 + 0.3 * S_PX, 100.0))  # camera moved
    assert result.frames == 1
    assert result.u == pytest.approx(124.0)


def test_distance_is_filtered_too():
    median = CentreMedian(3, 0.25)
    for d in (300.0, 330.0, 310.0):
        result = median.update(nut(0.0, 0.0, d))
    assert result.distance_mm == 310.0


def test_window_of_one_is_no_filter():
    median = CentreMedian(1, 0.25)
    result = median.update(nut(12.3, 4.5))
    assert result.settled and (result.u, result.v) == (12.3, 4.5)


def test_empty_window_raises():
    with pytest.raises(ValueError):
        CentreMedian(0, 0.25)
