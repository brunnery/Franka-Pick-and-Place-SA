import pytest

from sa_tasks import run_record


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_timeline_marks_seconds_since_start_and_tells_the_listener():
    clock, heard = Clock(), []
    timeline = run_record.Timeline(lambda name, t: heard.append((name, t)), clock)
    clock.now = 101.234
    assert timeline.mark("iris_closed") == pytest.approx(1.23)
    clock.now = 103.0
    timeline.mark("iris_closed")
    assert timeline.marks == {"iris_closed": 1.23, "iris_closed_2": 3.0}
    assert heard == [("iris_closed", 1.23), ("iris_closed_2", 3.0)]


def test_config_values_by_dotted_key():
    cfg = {"place": {"screw": {"tighten": {"duration_ms": 7000}}}, "vision": {"settle_s": 0.2}}
    assert run_record.config_values(cfg, ["place.screw.tighten", "vision.settle_s"]) == {
        "place.screw.tighten": {"duration_ms": 7000}, "vision.settle_s": 0.2}
    with pytest.raises(KeyError):
        run_record.config_values(cfg, ["place.seat"])
