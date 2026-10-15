import dataclasses
import math

import cv2
import numpy as np
import pytest

from sa_vision.detection import (NutParams, caliper_widths, classify_size, clip_roi,
                                 compactness, detect_nut, estimate_background, failure_reason,
                                 hexagon_area, mm_to_px)

FOCAL_PX = 2080.0
DISTANCE_MM = 340.0
WIDTH, HEIGHT = 1280, 720
SUBPIXEL = 4  # fixed-point bits for anti-aliased drawing
SCALE = FOCAL_PX / DISTANCE_MM  # px per mm on the nut plane

# Table: grey gradient like the real images. Nut: bright (specular) body, dark thread
# hole. DARK_FACET is darker than the table but not dark enough for the core.
TABLE_LEFT, TABLE_RIGHT = 190, 225
BRIGHT_FACET, DARK_FACET, HOLE = 250, 150, 100
HOLE_DIAMETER_RATIO = 0.55  # visible dark core / s
# Measured s is too large by about 2.5 px: OpenCV's anti-aliased polygons are about 1 px
# wider than exact, and the blur widens high-contrast edges by about 1 px.
ACROSS_FLATS_TOLERANCE_MM = 3.0 / SCALE

PARAMS = NutParams(
    blur_kernel_px=5, morph_kernel_px=7, morph_close_iterations=2, morph_open_iterations=1,
    roi=(0, 0, 0, 0), background_downscale=8, background_window_ratio=1.25,
    brighter_threshold=0.06, darker_threshold=0.25, core_threshold=0.28,
    across_flats_range_mm=(7.0, 19.0), target_across_flats_mm=0.0, size_tolerance=0.15,
    size_class_tolerance=0.05, min_width_ratio=0.78, max_width_ratio=0.95,
    min_hole_ratio=0.15, max_hole_ratio=0.85, max_centre_offset_ratio=0.25)


def with_params(**changes) -> NutParams:
    return dataclasses.replace(PARAMS, **changes)


def table() -> np.ndarray:
    row = np.linspace(TABLE_LEFT, TABLE_RIGHT, WIDTH)
    return np.tile(np.round(row).astype(np.uint8), (HEIGHT, 1))


def fill(image, points, value):
    cv2.fillPoly(image, [np.round(np.asarray(points) * 2 ** SUBPIXEL).astype(np.int32)], value,
                 cv2.LINE_AA, shift=SUBPIXEL)


def hexagon_corners(centre, across_flats_px, angle_deg=0.0):
    radius = across_flats_px / math.sqrt(3.0)  # circumradius
    angles = np.deg2rad(angle_deg + 60.0 * np.arange(6))
    return np.stack([centre[0] + radius * np.cos(angles), centre[1] + radius * np.sin(angles)],
                    axis=1)


def draw_circle(image, centre, diameter_px, value):
    c = tuple(int(round(x * 2 ** SUBPIXEL)) for x in centre)
    cv2.circle(image, c, int(round(diameter_px / 2 * 2 ** SUBPIXEL)), value, -1,
               cv2.LINE_AA, shift=SUBPIXEL)


def draw_nut(image, centre, across_flats_mm=13.0, angle_deg=0.0, hole=True,
             hole_offset_px=(0.0, 0.0), hole_ratio=HOLE_DIAMETER_RATIO, lower_half=BRIGHT_FACET):
    s_px = across_flats_mm * SCALE
    corners = hexagon_corners(centre, s_px, angle_deg)
    fill(image, corners, lower_half)
    fill(image, corners[[3, 4, 5, 0]], BRIGHT_FACET)  # upper half
    if hole:
        draw_circle(image, (centre[0] + hole_offset_px[0], centre[1] + hole_offset_px[1]),
                    hole_ratio * s_px, HOLE)


def nut_image(*centres, across_flats_mm=13.0):
    image = table()
    for centre in centres:
        draw_nut(image, centre, across_flats_mm)
    return image


@pytest.mark.parametrize("centre, angle", [((640.0, 360.0), 0.0), ((300.3, 200.7), 17.0),
                                           ((1000.5, 600.25), 42.0)])
def test_centre_and_size_of_single_nut(centre, angle):
    image = table()
    draw_nut(image, centre, angle_deg=angle)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert result.valid, result.status
    assert (result.u, result.v) == pytest.approx(centre, abs=0.5)
    assert result.across_flats_mm == pytest.approx(13.0, abs=ACROSS_FLATS_TOLERANCE_MM)
    assert result.size == "M8"
    assert result.width_ratio == pytest.approx(math.sqrt(3.0) / 2.0, abs=0.03)
    assert result.background == "estimated"


@pytest.mark.parametrize("across_flats_mm, size", [(7.0, "M4"), (10.0, "M6"), (13.0, "M8"),
                                                   (16.0, "M10"), (19.0, "M12")])
def test_size_independent(across_flats_mm, size):
    result = detect_nut(nut_image((640.0, 360.0), across_flats_mm=across_flats_mm),
                        FOCAL_PX, DISTANCE_MM, PARAMS)
    assert result.valid, result.status
    assert result.across_flats_mm == pytest.approx(across_flats_mm,
                                                   abs=ACROSS_FLATS_TOLERANCE_MM)
    assert result.size == size


def test_bright_and_dark_facets_form_one_silhouette():
    # Edges with less contrast cross the threshold further inside after the blur. The
    # dark half lies just beyond its threshold, so the centre moves towards the bright
    # half by about 1 px (0.16 mm). Uniform nuts in the other tests have no such bias.
    image = table()
    draw_nut(image, (640.0, 360.0), lower_half=DARK_FACET)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert result.valid, result.status
    assert len(result.candidates) == 1
    assert (result.u, result.v) == pytest.approx((640.0, 360.0), abs=1.5)
    assert result.size == "M8"


def cast_soft_shadow(image, centre, depth, across_flats_mm=13.0):
    """Darken the table by up to depth on the upper right of the nut (real: 0.2-0.4)."""
    s_px = across_flats_mm * SCALE
    shade = np.zeros(image.shape, np.float32)
    shadow_centre = (int(round(centre[0] + 0.25 * s_px)), int(round(centre[1] - 0.35 * s_px)))
    cv2.circle(shade, shadow_centre, int(round(0.6 * s_px)), 1.0, -1)
    shade = cv2.GaussianBlur(shade, (0, 0), 0.2 * s_px)
    image[:] = np.round(image * (1.0 - depth * shade)).astype(np.uint8)


@pytest.mark.parametrize("use_reference", [False, True])
def test_soft_shadow_does_not_enlarge_the_silhouette(use_reference):
    # The local median only partly absorbs the shadow: this synthetic shadow is rejected
    # (width ratio) from a depth of about 0.3 on. Real shadows of 0.4 still passed on the
    # test images of 15.10.; their shape is softer.
    image = table()
    cast_soft_shadow(image, (640.0, 360.0), depth=0.25)
    draw_nut(image, (640.0, 360.0))
    reference = table() if use_reference else None
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS, background=reference)
    assert result.valid, result.status
    assert result.across_flats_mm == pytest.approx(13.0, abs=ACROSS_FLATS_TOLERANCE_MM)
    assert result.width_ratio == pytest.approx(math.sqrt(3.0) / 2.0, abs=0.04)


def test_bgr_input_and_noise():
    rng = np.random.default_rng(0)
    image = nut_image((700.0, 300.0))
    noisy = np.clip(image + rng.normal(0, 8, image.shape), 0, 255).astype(np.uint8)
    result = detect_nut(cv2.cvtColor(noisy, cv2.COLOR_GRAY2BGR), FOCAL_PX, DISTANCE_MM, PARAMS)
    assert result.valid, result.status
    assert (result.u, result.v) == pytest.approx((700.0, 300.0), abs=0.7)


def test_empty_table_with_gradient_has_no_candidates():
    result = detect_nut(table(), FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert result.status == "no nut found"
    assert result.candidates == []


def test_estimated_background_follows_gradient_and_ignores_nut():
    image = nut_image((640.0, 360.0), across_flats_mm=19.0)  # largest nut
    window_px = 1.25 * mm_to_px(19.0 * 1.15, FOCAL_PX, DISTANCE_MM)
    background = estimate_background(image, window_px, 8)
    assert np.abs(background.astype(int) - table().astype(int)).max() <= 3


def test_hexagon_without_hole_is_rejected():
    image = table()
    draw_nut(image, (640.0, 360.0), hole=False)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["no hole"]


def test_square_stud_base_is_rejected_by_width_ratio():
    image = table()
    side = 10.0 * SCALE
    fill(image, [(600, 300), (600 + side, 300), (600 + side, 300 + side), (600, 300 + side)], 40)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["width ratio"]
    assert result.candidates[0].width_ratio == pytest.approx(1 / math.sqrt(2.0), abs=0.03)


def test_round_washer_is_rejected_by_width_ratio():
    image = table()
    draw_circle(image, (640.0, 360.0), 16.0 * SCALE, BRIGHT_FACET)
    draw_circle(image, (640.0, 360.0), 8.4 * SCALE, HOLE)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["width ratio"]


def test_eccentric_hole_is_rejected():
    image = table()
    draw_nut(image, (640.0, 360.0), hole_offset_px=(0.2 * 13.0 * SCALE, 0.0), hole_ratio=0.45)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, with_params(max_centre_offset_ratio=0.1))
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["centre offset"]


def test_too_small_hole_is_rejected():
    image = table()
    draw_nut(image, (640.0, 360.0), hole_ratio=0.25)  # core area ratio 0.06
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["hole ratio"]


def test_object_outside_size_range_is_rejected():
    result = detect_nut(nut_image((640.0, 360.0), across_flats_mm=30.0),
                        FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["size"]


def test_two_nuts_without_target_are_ambiguous():
    result = detect_nut(nut_image((400.0, 360.0), (900.0, 360.0)), FOCAL_PX, DISTANCE_MM, PARAMS)
    assert not result.valid
    assert result.status.startswith("2 nuts found")


def test_target_size_selects_the_m8_nut():
    image = table()
    draw_nut(image, (400.0, 360.0), across_flats_mm=19.0)
    draw_nut(image, (900.0, 360.0), across_flats_mm=13.0)
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, with_params(target_across_flats_mm=13.0))
    assert result.valid, result.status
    assert (result.u, result.v) == pytest.approx((900.0, 360.0), abs=0.5)
    assert result.size == "M8"
    assert sorted(c.rejected for c in result.candidates) == ["", "size"]


def test_reference_image_removes_static_objects():
    reference = nut_image((400.0, 360.0))  # a fixed nut-like object in the empty work area
    image = nut_image((400.0, 360.0), (900.0, 360.0))
    assert not detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS).valid
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS, background=reference)
    assert result.valid, result.status
    assert result.background == "reference"
    assert (result.u, result.v) == pytest.approx((900.0, 360.0), abs=0.5)


def test_reference_image_of_other_size_raises():
    with pytest.raises(ValueError):
        detect_nut(table(), FOCAL_PX, DISTANCE_MM, PARAMS, background=np.zeros((480, 640)))


def test_roi_excludes_second_nut_and_keeps_full_image_coordinates():
    params = with_params(roi=(700, 200, 400, 300))
    result = detect_nut(nut_image((400.0, 360.0), (900.0, 360.0)), FOCAL_PX, DISTANCE_MM, params)
    assert result.valid, result.status
    assert (result.u, result.v) == pytest.approx((900.0, 360.0), abs=0.5)
    assert result.mask.shape == (300, 400)


HAND_HELD_RANGE_MM = (200.0, 500.0)
M8_TARGET = with_params(target_across_flats_mm=13.0)


def test_distance_from_size_of_the_target_nut():
    # distance_mm is ignored in this mode; the nut is drawn as seen from 340 mm
    result = detect_nut(nut_image((640.0, 360.0)), FOCAL_PX, None, M8_TARGET,
                        distance_range_mm=HAND_HELD_RANGE_MM)
    assert result.valid, result.status
    assert result.distance_from_size
    assert result.distance_mm == pytest.approx(DISTANCE_MM, abs=3.0 / SCALE * DISTANCE_MM / 13.0)
    assert result.across_flats_mm == 13.0  # assumed, not measured
    assert result.size == ""


def test_distance_from_size_rejects_implausible_distance():
    result = detect_nut(nut_image((640.0, 360.0)), FOCAL_PX, None, M8_TARGET,
                        distance_range_mm=(400.0, 500.0))  # nut implies d = 340 mm
    assert not result.valid
    assert [c.rejected for c in result.candidates] == ["size"]
    assert failure_reason(result).startswith("wrong size (d would be 3")


def test_distance_from_size_cannot_tell_nut_sizes_apart():
    # Limitation: without a known d a M12 nut looks like a closer M8 nut
    image = table()
    draw_nut(image, (400.0, 360.0), across_flats_mm=13.0)
    draw_nut(image, (900.0, 360.0), across_flats_mm=18.0)
    result = detect_nut(image, FOCAL_PX, None, M8_TARGET, distance_range_mm=HAND_HELD_RANGE_MM)
    assert not result.valid
    assert failure_reason(result) == "2 candidates"


def test_distance_from_size_needs_a_target():
    with pytest.raises(ValueError):
        detect_nut(table(), FOCAL_PX, None, PARAMS, distance_range_mm=HAND_HELD_RANGE_MM)


def test_failure_reasons():
    assert failure_reason(detect_nut(nut_image((640.0, 360.0)), FOCAL_PX, DISTANCE_MM,
                                     PARAMS)) == ""
    assert failure_reason(detect_nut(table(), FOCAL_PX, DISTANCE_MM, PARAMS)) == "nothing in view"
    image = table()
    draw_nut(image, (640.0, 360.0), hole=False)
    assert failure_reason(detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)) == "no hole"
    wrong = detect_nut(nut_image((640.0, 360.0), across_flats_mm=30.0), FOCAL_PX, DISTANCE_MM,
                       PARAMS)
    assert failure_reason(wrong).startswith("wrong size (s = 3")


def test_failure_reason_prefers_the_candidate_that_got_furthest():
    # A large dark blob (wrong size) and a hexagon without hole: the hexagon decides
    image = table()
    fill(image, [(50, 50), (450, 50), (450, 350), (50, 350)], 40)
    draw_nut(image, (900.0, 360.0), hole=False)
    assert failure_reason(detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)) == "no hole"


@pytest.mark.parametrize("across_flats_mm, size", [
    (7.0, "M4"), (8.1, "M5"), (13.0, "M8"), (13.6, "M8"), (16.0, "M10"), (17.0, "M10"),
    (18.2, "M12"), (19.0, "M12"),
    (17.5, ""),   # between M10 DIN 934 (17) and M12 ISO 4032 (18)
    (14.6, ""),   # no standard size within 5 %
    (7.5, ""),    # between M4 and M5
])
def test_classify_size(across_flats_mm, size):
    assert classify_size(across_flats_mm, 0.05) == size


def test_caliper_widths_of_hexagon_and_square():
    hexagon = hexagon_corners((0.0, 0.0), 100.0, 13.0).reshape(-1, 1, 2)
    s, e = caliper_widths(hexagon)
    assert s == pytest.approx(100.0)
    assert e == pytest.approx(200.0 / math.sqrt(3.0))
    square = np.array([[[0, 0]], [[100, 0]], [[100, 100]], [[0, 100]]], np.float32)
    assert caliper_widths(square) == pytest.approx((100.0, 100.0 * math.sqrt(2.0)))


def test_hexagon_area_and_scaling():
    assert hexagon_area(13.0) == pytest.approx(146.4, abs=0.1)
    assert mm_to_px(13.0, FOCAL_PX, 300.0) == pytest.approx(2 * mm_to_px(13.0, FOCAL_PX, 600.0))


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
