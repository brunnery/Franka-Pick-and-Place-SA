import numpy as np

from sa_vision.debug_draw import (AMBIGUOUS_COLOUR, FAILURE_COLOUR, NUT_COLOUR, draw_debug,
                                  info_line, nut_label)
from sa_vision.detection import detect_nut
from test_detection import (DISTANCE_MM, FOCAL_PX, M8_TARGET, HAND_HELD_RANGE_MM, PARAMS,
                            nut_image, table)

PRINCIPAL_POINT = (695.4, 405.6)


def colour_pixels(image, colour):
    return int(np.all(image == np.array(colour, np.uint8), axis=2).sum())


def bgr(gray):
    return np.repeat(gray[:, :, None], 3, axis=2)


def test_detected_nut_is_drawn_green_with_offset_text():
    image = bgr(nut_image((900.0, 300.0)))
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    out = draw_debug(image, result, PRINCIPAL_POINT, 15.0, (12.3, -4.5))
    assert out.shape == image.shape and not np.array_equal(out, image)
    assert colour_pixels(out, NUT_COLOUR) > 100
    assert nut_label(result, (12.3, -4.5)) == [
        f"s = {result.across_flats_mm:.1f} mm M8", "d = 340 mm", "dx_robo = +12.3 mm",
        "dy_robo = -4.5 mm"]


def test_missing_nut_is_reported_in_red():
    image = bgr(table())
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    out = draw_debug(image, result, PRINCIPAL_POINT)
    assert colour_pixels(out, FAILURE_COLOUR) > 100
    assert colour_pixels(out, NUT_COLOUR) == 0


def test_ambiguous_candidates_are_drawn_yellow():
    image = bgr(nut_image((300.0, 300.0), (1000.0, 300.0)))
    result = detect_nut(image, FOCAL_PX, DISTANCE_MM, PARAMS)
    out = draw_debug(image, result, PRINCIPAL_POINT)
    assert colour_pixels(out, AMBIGUOUS_COLOUR) > 200
    assert colour_pixels(out, NUT_COLOUR) == 0


def test_info_line_names_method_rate_and_distance_source():
    fixed = detect_nut(table(), FOCAL_PX, DISTANCE_MM, PARAMS)
    assert info_line(fixed, 14.94) == "A (local median) | 14.9 fps | d = 340 mm (yaml)"
    estimated = detect_nut(nut_image((640.0, 360.0)), FOCAL_PX, None, M8_TARGET,
                           background=table(), distance_range_mm=HAND_HELD_RANGE_MM)
    assert info_line(estimated, None) == "A' (reference image) | offline | d estimated from size"
    assert nut_label(estimated, None)[0] == "s = 13.0 mm (assumed)"
