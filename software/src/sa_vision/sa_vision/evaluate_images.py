"""Run the nut detection offline on saved images (no ROS needed).

  ros2 run sa_vision evaluate_images data/images/*.png --debug-dir /tmp/debug
  [--params software/src/sa_vision/config/vision.yaml]
  [--calibration calibration/hd_camera.yaml] [--distance-mm 300]

Prints one line per image and writes the debug overlay (same as /vision/debug)
plus the binary mask next to it.
"""
import argparse
from pathlib import Path

import cv2
import yaml

from sa_vision import geometry
from sa_vision.debug_draw import draw_debug
from sa_vision.detection import NutParams, detect_nut

# Paths relative to the repository root
DEFAULT_PARAMS = Path("software/src/sa_vision/config/vision.yaml")
DEFAULT_CALIBRATION = Path("calibration/hd_camera.yaml")
# Rejected contours smaller than this fraction of the expected area are not listed
REPORT_MIN_AREA_RATIO = 0.25


def load_params(path: Path):
    params = yaml.safe_load(path.read_text())["vision_node"]["ros__parameters"]
    nut = dict(params["nut"])
    nut["roi"] = tuple(nut["roi"])
    return NutParams(**nut), params["object_plane_distance_mm"]


def load_calibration(path: Path):
    calibration = yaml.safe_load(path.read_text())
    k = geometry.camera_matrix(calibration["camera_matrix"]["data"])
    return k, calibration["distortion_coefficients"]["data"]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--distance-mm", type=float, help="override object_plane_distance_mm")
    parser.add_argument("--debug-dir", type=Path, help="write overlays and masks here")
    args = parser.parse_args()

    params, distance_mm = load_params(args.params)
    if args.distance_mm is not None:
        distance_mm = args.distance_mm
    k, dist = load_calibration(args.calibration)
    focal_px = 0.5 * (k[0, 0] + k[1, 1])
    if args.debug_dir:
        args.debug_dir.mkdir(parents=True, exist_ok=True)

    print(f"d = {distance_mm:.0f} mm, f = {focal_px:.0f} px, params: {args.params}")
    print(f"{'image':40s} {'valid':5s} {'u':>7s} {'v':>7s} {'A/A_exp':>7s} {'C':>5s}  status / rejections")
    num_valid = 0
    for path in sorted(args.images):
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path.name:40s} cannot read")
            continue
        result = detect_nut(image, focal_px, distance_mm, params)
        num_valid += result.valid
        min_area = REPORT_MIN_AREA_RATIO * result.expected_area_px
        big = [c for c in result.candidates if c.area_px >= min_area]
        reasons = ", ".join(f"{c.rejected}({c.area_px / result.expected_area_px:.2f})"
                            for c in big if c.rejected)
        print(f"{path.name:40s} {str(result.valid):5s} {result.u:7.1f} {result.v:7.1f} "
              f"{result.area_px / result.expected_area_px:7.2f} {result.compactness:5.2f}  "
              f"{result.status}" + (f" | {reasons}" if reasons else ""))
        if args.debug_dir:
            label = ""
            if result.valid:
                uu, vu = geometry.undistort_pixel(result.u, result.v, k, dist)
                p = geometry.pixel_to_camera_point(uu, vu, k, distance_mm)
                label = f"P_C = ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm"
            overlay = draw_debug(image, result, (k[0, 2], k[1, 2]), label)
            cv2.imwrite(str(args.debug_dir / path.name), overlay)
            if result.mask is not None:
                cv2.imwrite(str(args.debug_dir / f"{path.stem}_mask.png"), result.mask)
    print(f"valid: {num_valid}/{len(args.images)}")


if __name__ == "__main__":
    main()
