from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from sa_vision.vision_node import (CHOSEN, Candidate, Polygon, Tracker, choose, find_objects,
                                   fit_polygon, rank)

YAML = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" /
                       "vision.yaml").read_text())["vision_node"]["ros__parameters"]
NUT = {**YAML["detection"], **YAML["targets"]["nut"]}
STUD = {**YAML["detection"], **YAML["targets"]["stud"]}


def polygon_corners(centre, s, angle_deg, sides):
    return Polygon(centre, s, angle_deg, sides).corners()


@pytest.mark.parametrize("sides, angle", [(6, 17.0), (4, 61.0)])
def test_fit_polygon_recovers_centre_and_size(sides, angle):
    polygon = fit_polygon(polygon_corners((400.0, 300.0), 80.0, angle, sides), sides, 1.0)
    assert polygon.centre == pytest.approx((400.0, 300.0), abs=0.5)
    assert polygon.across_flats_px == pytest.approx(80.0, rel=0.02)


def test_nut_on_a_shaded_table_is_found_and_a_lying_bolt_rejected():
    image = np.tile(np.linspace(160, 200, 1280).astype(np.uint8), (720, 1))  # illumination
    cv2.rectangle(image, (200, 300), (420, 340), 60, -1)                     # lying bolt
    cv2.fillPoly(image, [np.round(polygon_corners((800.3, 360.6), 66.0, 10.0, 6)).astype(np.int32)],
                 60)
    _, objects = find_objects(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), NUT)
    assert choose(objects, NUT, 66.0).startswith("ok")
    chosen = next(c for c in objects if c.verdict == CHOSEN)
    assert chosen.polygon.centre == pytest.approx((800.3, 360.6), abs=1.5)


def test_standing_stud_is_ranked_first_and_square():
    image = np.full((720, 1280, 3), 190, np.uint8)
    cv2.rectangle(image, (600, 330), (660, 390), (60, 60, 60), -1)  # base, thread inside
    cv2.circle(image, (900, 200), 9, (60, 60, 60), -1)               # a speck
    _, objects = find_objects(image, STUD)
    mm_per_px = 10.0 / 60.0
    ranked = rank(objects, STUD, 60.0, mm_per_px)
    assert len(ranked) == 1 and ranked[0].shape_ok
    assert ranked[0].polygon.centre == pytest.approx((630.0, 360.0), abs=1.5)


def test_tracker_takes_the_median_and_never_confirms_a_flickering_speck():
    t = YAML["tracking"]
    tracker = Tracker(t["frames"], t["match_radius_px"], t["max_misses"])
    for i, centre in enumerate([(500, 300), (501, 300), (499, 301), (512, 290), (500, 299)]):
        objects = [Candidate(None, Polygon(centre, 50.0, 0.0, 4), 0.7)]
        if i % 2 == 0:
            objects.append(Candidate(None, Polygon((900.0, 100.0), 10.0, 0.0, 4), 0.5))
        tracked = tracker.update(objects)
    confirmed = [c for c in tracked if c.seen >= 5]
    assert len(confirmed) == 1 and confirmed[0].polygon.centre == pytest.approx((500.0, 300.0))
