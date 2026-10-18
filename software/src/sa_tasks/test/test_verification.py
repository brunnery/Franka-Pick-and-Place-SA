import numpy as np
import pytest

from sa_tasks.verification import correction


def test_constant_offset_is_the_mean_and_leaves_no_rest():
    computed = np.array([[500.0, -230.0], [480.0, -250.0], [520.0, -210.0]])
    result = correction(computed + [2.0, -3.0], computed)
    assert result["mean"] == pytest.approx([2.0, -3.0])
    assert result["std"] == pytest.approx([0.0, 0.0])
    assert result["norm_after"].max() == pytest.approx(0.0)


def test_spread_is_reported():
    computed = np.zeros((2, 2))
    result = correction(np.array([[1.0, 0.0], [3.0, 0.0]]), computed)
    assert result["mean"] == pytest.approx([2.0, 0.0])
    assert result["norm_after"] == pytest.approx([1.0, 1.0])
