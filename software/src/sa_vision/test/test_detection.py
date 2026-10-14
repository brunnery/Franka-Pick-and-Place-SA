import math

import cv2
import numpy as np
import pytest

from sa_vision.detection import (NutParams, clip_roi, compactness, detect_nut,
                                 expected_area_px, hexagon_area)

FOCAL_PX = 2080.0
DISTANCE_MM = 300.0
WIDTH, HEIGHT = 1280, 720
SUBPIXEL = 4  # fixed-point bits for anti-aliased drawing

PARAMS = NutParams(
    blur_kernel_px=5, morph_kernel_px=5, morph_open_iterations=1, morph_close_iterations=1,
    roi=(0, 0, 0, 0), across_flats_mm=10.0, hole_diameter_mm=5.0, area_tolerance=0.35,
    hole_ratio_tolerance=0.5, min_compactness=0.75, max_centre_offset_ratio=0.15,
    centre_source="hole")


def blank(value=235):
    return np.full((HEIGHT, WIDTH), value, np.uint8)


def draw_hexagon(image, centre, across_flats_px, angle_deg=0.0, value=60):
    radius = across_flats_px / math.sqrt(3.0)  # circumradius
    angles = np.deg2rad(angle_deg + 60.0 * np.arange(6))
    corners = np.stack([centre[0] + radius * np.cos(angles),
                        centre[1] + radius * np.sin(angles)], axis=1)
    cv2.fillPoly(image, [np.round(corners * 2 ** SUBPIXEL).astype(np.int32)], value,
                 cv2.LINE_AA, shift=SUBPIXEL)


def draw_circle(image, centre, diameter_px, value):
    c = tuple(int(round(x * 2 ** SUBPIXEL)) for x in centre)
    cv2.circle(image, c, int(round(diameter_px / 2 * 2 ** SUBPIXEL)), value, -1,
               cv2.LINE_AA, shift=SUBPIXEL)


def draw_nut(image, centre, distance_mm=DISTANCE_MM, angle_deg=0.0, hole=True):
    scale = FOCAL_PX / distance_mm
    draw_hexagon(image, centre, 10.0 * scale, angle_deg)
    if hole:
        draw_circle(image, centre, 5.0 * scale, int(image[0, 0]))


@pytest.mark.parametrize("centre, angle", [((640.0, 360.0), 0.0), ((300.3, 200.7), 17.0),
                                           ((1000.5, 600.25), 42.0)])
def test_centre_of_single_nut(centre, angle):
    image = blank()
    draw_nut(image, centre, angle_deg=angle)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert result.valid, result.status
    assert result.u == pytest.approx(centre[0], abs=0.3)
    assert result.v == pytest.approx(centre[1], abs=0.3)
    assert result.area_px == pytest.approx(result.expected_area_px, rel=0.05)


def test_outer_centre_source_matches_hole_centre():
    image = blank()
    draw_nut(image, (500.0, 400.0))
    params = NutParams(**{**PARAMS.__dict__, "centre_source": "outer"})
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, params)
    assert result.valid
    assert (result.u, result.v) == pytest.approx((500.0, 400.0), abs=0.3)


def test_bgr_input_and_noise():
    rng = np.random.default_rng(0)
    image = blank()
    draw_nut(image, (700.0, 300.0))
    noisy = np.clip(image + rng.normal(0, 8, image.shape), 0, 255).astype(np.uint8)
    result = detect_nut(cv2.cvtColor(noisy, cv2.COLOR_GRAY2BGR), FOCAL_PX, DISTANCE_MM, PARAMS)
    assert result.valid, result.status
    assert (result.u, result.v) == pytest.approx((700.0, 300.0), abs=0.5)


def test_empty_image_is_invalid():
    result = detect_nut(blank(), FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert result.status == "no nut found"


def test_filled_hexagon_without_hole_is_rejected():
    image = blank()
    draw_nut(image, (640.0, 360.0), hole=False)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["no hole"]


def test_wrong_size_is_rejected():
    image = blank()
    draw_nut(image, (640.0, 360.0), distance_mm=DISTANCE_MM / 2)  # appears twice as large
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["area"]


def test_washer_like_ring_is_rejected_by_hole_ratio():
    image = blank()
    scale = FOCAL_PX / DISTANCE_MM
    draw_hexagon(image, (640.0, 360.0), 10.0 * scale)
    draw_circle(image, (640.0, 360.0), 8.5 * scale, 235)  # thin ring, hole far too large
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert result.candidates[0].rejected in ("hole ratio", "area")


def test_eccentric_hole_is_rejected():
    image = blank()
    scale = FOCAL_PX / DISTANCE_MM
    draw_hexagon(image, (640.0, 360.0), 10.0 * scale)
    draw_circle(image, (640.0 + 2.0 * scale, 360.0), 5.0 * scale, 235)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert result.candidates[0].rejected == "centre offset"


def test_two_nuts_are_ambiguous():
    image = blank()
    draw_nut(image, (400.0, 360.0))
    draw_nut(image, (900.0, 360.0))
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert result.status == "2 nuts found"


def test_roi_excludes_second_nut_and_keeps_full_image_coordinates():
    image = blank()
    draw_nut(image, (400.0, 360.0))
    draw_nut(image, (900.0, 360.0))
    params = NutParams(**{**PARAMS.__dict__, "roi": (700, 200, 400, 300)})
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, params)
    assert result.valid, result.status
    assert (result.u, result.v) == pytest.approx((900.0, 360.0), abs=0.3)
    assert result.mask.shape == (300, 400)


def test_invalid_centre_source_raises():
    params = NutParams(**{**PARAMS.__dict__, "centre_source": "middle"})
    with pytest.raises(ValueError):
        detect_nut(blank(), FOCAL_PX, DISTANCE_MM, params)


def test_expected_area_scales_with_inverse_square_distance():
    a300 = expected_area_px(hexagon_area(10.0), FOCAL_PX, 300.0)
    a600 = expected_area_px(hexagon_area(10.0), FOCAL_PX, 600.0)
    assert hexagon_area(10.0) == pytest.approx(86.6, abs=0.1)
    assert a300 == pytest.approx(4 * a600)


def test_compactness_of_reference_shapes():
    image = np.zeros((600, 600), np.uint8)
    cv2.circle(image, (300, 300), 200, 255, -1)
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    assert compactness(contours[0]) == pytest.approx(1.0, abs=0.12)  # pixel perimeter bias
    square = np.array([[[0, 0]], [[100, 0]], [[100, 100]], [[0, 100]]], np.int32)
    assert compactness(square) == pytest.approx(math.pi / 4)


@pytest.mark.parametrize("roi, expected", [
    ((0, 0, 0, 0), (0, 0, 1280, 720)),
    ((100, 50, 200, 100), (100, 50, 200, 100)),
    ((-10, -10, 50, 50), (0, 0, 40, 40)),
    ((1200, 700, 200, 200), (1200, 700, 80, 20)),
])
def test_clip_roi(roi, expected):
    assert clip_roi(roi, (HEIGHT, WIDTH)) == expected
