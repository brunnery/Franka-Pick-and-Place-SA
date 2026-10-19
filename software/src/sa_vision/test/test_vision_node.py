import math

import cv2
import numpy as np
import pytest
import yaml
from pathlib import Path

from sa_vision.vision_node import CHOSEN, REJECTED_SHAPE, find_object, fit_hexagon

PARAMS = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" /
                         "vision.yaml").read_text())["vision_node"]["ros__parameters"]["detection"]


def hexagon_corners(centre, s, angle_deg):
    r = s / math.sqrt(3.0)
    a = np.radians(angle_deg + 30.0 + 60.0 * np.arange(6))
    return np.stack([centre[0] + r * np.cos(a), centre[1] + r * np.sin(a)], axis=1)


@pytest.mark.parametrize("angle", [0.0, 17.0, 44.0])
def test_fit_hexagon_recovers_centre_size_angle(angle):
    hexagon = fit_hexagon(hexagon_corners((400.0, 300.0), 80.0, angle), 1.0)
    assert hexagon.centre == pytest.approx((400.0, 300.0), abs=0.5)
    assert hexagon.across_flats_px == pytest.approx(80.0, rel=0.02)
    assert hexagon.regularity < 0.01


@pytest.mark.parametrize("value", [60, 250])  # darker or brighter than the background
def test_finds_one_object_on_a_blank_background(value):
    image = np.full((720, 1280), 180, np.uint8)
    image[:, :] = np.linspace(160, 200, 1280).astype(np.uint8)  # illumination gradient
    corners = hexagon_corners((700.3, 350.6), 66.0, 10.0)
    cv2.fillPoly(image, [np.round(corners * 16).astype(np.int32)], value, cv2.LINE_AA, shift=4)
    hexagon, _, status = find_object(image, PARAMS)
    assert status == "ok"
    assert hexagon.centre == pytest.approx((700.3, 350.6), abs=1.0)


def test_square_still_gives_its_centre():
    image = np.full((720, 1280), 190, np.uint8)
    cv2.rectangle(image, (500, 300), (560, 360), 60, -1)
    hexagon, _, _ = find_object(image, PARAMS)
    assert hexagon.centre == pytest.approx((530.0, 330.0), abs=1.5)


def test_blank_image_finds_nothing():
    hexagon, _, status = find_object(np.full((720, 1280), 190, np.uint8), PARAMS)
    assert hexagon is None and status.startswith("nothing")


def test_several_objects_shape_filter_rejects_a_bolt():
    image = np.full((720, 1280), 185, np.uint8)
    cv2.rectangle(image, (200, 300), (420, 340), 60, -1)     # lying bolt, larger than the nut
    corners = hexagon_corners((800.0, 360.0), 66.0, 10.0)
    cv2.fillPoly(image, [np.round(corners).astype(np.int32)], 60)
    objects = []
    hexagon, _, status = find_object(image, PARAMS, None, 66.0, objects)
    assert status.startswith("ok")
    assert hexagon.centre == pytest.approx((800.0, 360.0), abs=1.5)
    assert sorted(o.verdict for o in objects) == sorted([CHOSEN, REJECTED_SHAPE])


def test_several_nuts_size_filter_takes_the_target_size():
    image = np.full((720, 1280), 185, np.uint8)
    for centre, s in (((400.0, 360.0), 100.0), ((900.0, 360.0), 66.0)):
        corners = hexagon_corners(centre, s, 10.0)
        cv2.fillPoly(image, [np.round(corners).astype(np.int32)], 60)
    hexagon, _, status = find_object(image, PARAMS, None, 66.0)
    assert status.startswith("ok")
    assert hexagon.centre == pytest.approx((900.0, 360.0), abs=1.5)
