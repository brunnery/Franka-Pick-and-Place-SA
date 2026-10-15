"""Run the nut detection offline on saved images (no ROS needed).

  ros2 run sa_vision evaluate_images data/images/*.png --debug-dir /tmp/debug
  [--params software/src/sa_vision/config/vision.yaml]
  [--calibration calibration/hd_camera.yaml] [--distance-mm 300]
  [--reference <png>] [--target-mm 13]

Prints one line per image and writes the debug overlay (same as /vision/debug)
plus the binary mask next to it. Without a reference image (reference_image of
vision.yaml or --reference) the background is estimated from each image.
"""
import argparse
import dataclasses
from pathlib import Path

import cv2
import yaml

from sa_vision import geometry
from sa_vision.debug_draw import draw_debug
from sa_vision.detection import detect_nut
from sa_vision.params import load_vision_config

# Paths relative to the repository root
DEFAULT_PARAMS = Path("software/src/sa_vision/config/vision.yaml")
DEFAULT_CALIBRATION = Path("calibration/hd_camera.yaml")
# Rejected silhouettes with s below this fraction of the smallest nut are not listed
REPORT_MIN_SIZE_RATIO = 0.5


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
    parser.add_argument("--reference", type=Path, help="override reference_image")
    parser.add_argument("--target-mm", type=float,
                        help="override target_across_flats_mm (0 = any size)")
    parser.add_argument("--debug-dir", type=Path, help="write overlays and masks here")
    args = parser.parse_args()

    config = load_vision_config(args.params)
    params = config.nut
    if args.target_mm is not None:
        params = dataclasses.replace(params, target_across_flats_mm=args.target_mm)
    distance_mm = config.object_plane_distance_mm
    if args.distance_mm is not None:
        distance_mm = args.distance_mm
    reference_path = args.reference or Path(config.reference_image)
    background = cv2.imread(str(reference_path)) if reference_path.is_file() else None
    k, dist = load_calibration(args.calibration)
    focal_px = 0.5 * (k[0, 0] + k[1, 1])
    if args.debug_dir:
        args.debug_dir.mkdir(parents=True, exist_ok=True)

    print(f"d = {distance_mm:.0f} mm, f = {focal_px:.0f} px, "
          f"target s = {params.target_across_flats_mm:g} mm, params: {args.params}, "
          f"background: {reference_path if background is not None else 'estimated'}")
    print(f"{'image':32s} {'valid':5s} {'u':>7s} {'v':>7s} {'s_mm':>5s} {'size':4s} "
          f"{'s/e':>4s} {'hole':>4s}  status | rejected: reason(s_mm)")
    num_valid = 0
    min_report_mm = REPORT_MIN_SIZE_RATIO * params.across_flats_range_mm[0]
    for path in sorted(args.images):
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path.name:32s} cannot read")
            continue
        result = detect_nut(image, focal_px, distance_mm, params, background)
        num_valid += result.valid
        hole_ratio = result.hole_area_px / result.area_px if result.valid else float("nan")
        reasons = ", ".join(f"{c.rejected}({c.across_flats_mm:.1f})" for c in result.candidates
                            if c.rejected and c.across_flats_mm >= min_report_mm)
        print(f"{path.name:32s} {str(result.valid):5s} {result.u:7.1f} {result.v:7.1f} "
              f"{result.across_flats_mm:5.1f} {result.size:4s} {result.width_ratio:4.2f} "
              f"{hole_ratio:4.2f}  {result.status}" + (f" | {reasons}" if reasons else ""))
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
