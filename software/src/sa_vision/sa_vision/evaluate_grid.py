"""Detection rate and position accuracy on a bench set (printed grid, no ROS needed).

  ros2 run sa_vision evaluate_grid data/bench/diffuse [--debug-dir /tmp/grid_debug]
  [--grid data/grid/grid.yaml] [--params ...] [--calibration ...] [--no-reference]

The bench directory holds reference.png, grid/*.png, extra/*.png and labels.yaml
(grid image -> mark, extra image -> expected result). d for the size filter is
camera_to_table_mm - nut_height_mm of labels.yaml. Prints per grid mark the detected
centre and the position error after fitting the grid (THEORY 2.8), then the summary
and the result of the extra images. With --debug-dir also writes the debug overlays
and errors.png (error vectors over the image).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from sa_vision import geometry
from sa_vision.debug_draw import draw_debug
from sa_vision.detection import detect_nut, failure_reason
from sa_vision.evaluate_images import DEFAULT_CALIBRATION, DEFAULT_PARAMS, load_calibration
from sa_vision.grid_fit import homography_residuals_mm, similarity_residuals_mm
from sa_vision.params import load_vision_config

DEFAULT_GRID = Path("data/grid/grid.yaml")
EXPECT_VALID = {"none": False, "nut": True, "edge": False}
ARROW_SCALE = 50.0  # errors.png: 1 mm error drawn as 50 px


def detect(path, focal_px, distance_mm, params, background):
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(path)
    return image, detect_nut(image, focal_px, distance_mm, params, background)


def save_overlay(debug_dir, path, image, result, k):
    if debug_dir:
        cv2.imwrite(str(debug_dir / path.name), draw_debug(image, result, (k[0, 2], k[1, 2])))


def plot_errors(path, image_size, pixels, residuals_mm, marks, rms_mm):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    width, height = image_size
    fig, ax = plt.subplots(figsize=(8, 8 * height / width + 0.6))
    ax.quiver(pixels[:, 0], pixels[:, 1], residuals_mm[:, 0] * ARROW_SCALE,
              residuals_mm[:, 1] * ARROW_SCALE, angles="xy", scale_units="xy", scale=1)
    for (u, v), mark in zip(pixels, marks):
        ax.annotate(str(mark), (u, v), textcoords="offset points", xytext=(4, 4), fontsize=7)
    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_aspect("equal")
    ax.set_xlabel("u [px]")
    ax.set_ylabel("v [px]")
    ax.set_title(f"Position error after grid fit (arrow: 1 mm = {ARROW_SCALE:g} px), "
                 f"RMS {rms_mm:.2f} mm")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bench", type=Path)
    parser.add_argument("--grid", type=Path, default=DEFAULT_GRID)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--no-reference", action="store_true",
                        help="ignore reference.png (method A, local median only)")
    parser.add_argument("--debug-dir", type=Path)
    args = parser.parse_args()

    labels = yaml.safe_load((args.bench / "labels.yaml").read_text())
    grid = {m["id"]: (m["x_mm"], m["y_mm"]) for m in yaml.safe_load(args.grid.read_text())["marks"]}
    params = load_vision_config(args.params).nut
    k, dist = load_calibration(args.calibration)
    focal_px = 0.5 * (k[0, 0] + k[1, 1])
    distance_mm = labels["camera_to_table_mm"] - labels["nut_height_mm"]
    background = None if args.no_reference else cv2.imread(str(args.bench / "reference.png"))
    if args.debug_dir:
        args.debug_dir.mkdir(parents=True, exist_ok=True)

    grid_images = sorted((args.bench / "grid").glob("*.png"))
    if len(grid_images) != len(labels["grid_marks"]):
        parser.error(f"{len(grid_images)} grid images but {len(labels['grid_marks'])} marks")
    print(f"d = {distance_mm:.0f} mm (size filter), f = {focal_px:.0f} px, target s = "
          f"{params.target_across_flats_mm:g} mm, background: "
          f"{'estimated' if background is None else 'reference.png'}")

    found = []  # (mark, u, v, s_mm)
    image_size = None
    print(f"\n{'mark':>4s} {'valid':5s} {'u':>7s} {'v':>7s} {'s_mm':>5s}  status")
    for path, mark in zip(grid_images, labels["grid_marks"]):
        image, result = detect(path, focal_px, distance_mm, params, background)
        image_size = image.shape[1], image.shape[0]
        save_overlay(args.debug_dir, path, image, result, k)
        print(f"{mark:4d} {str(result.valid):5s} {result.u:7.1f} {result.v:7.1f} "
              f"{result.across_flats_mm:5.1f}  {result.status if result.valid else failure_reason(result)}")
        if result.valid:
            found.append((mark, result.u, result.v, result.across_flats_mm))

    print(f"\nGrid: {len(found)}/{len(grid_images)} nuts detected")
    if len(found) >= 4:
        marks = [f[0] for f in found]
        sheet = np.array([grid[m] for m in marks])
        pixels = np.array([(f[1], f[2]) for f in found])
        undistorted = np.array([geometry.undistort_pixel(u, v, k, dist) for u, v in pixels])
        normalised = (undistorted - k[[0, 1], [2, 2]]) / k[[0, 1], [0, 1]]
        d_fit, angle, residuals = similarity_residuals_mm(sheet, normalised)
        homography = homography_residuals_mm(sheet, normalised, d_fit)
        error = np.linalg.norm(residuals, axis=1)
        rms = float(np.sqrt((error ** 2).mean()))
        sizes = np.array([f[3] for f in found]) * d_fit / distance_mm
        print(f"Grid fit: d = {d_fit:.1f} mm to the nut centres (given {distance_mm:.0f}), "
              f"sheet rotated {angle:.2f} deg")
        print(f"\n{'mark':>4s} {'ex_mm':>6s} {'ey_mm':>6s} {'|e|':>5s} {'|e|_H':>6s}")
        for mark, (ex, ey), e, eh in zip(marks, residuals, error, np.linalg.norm(homography, axis=1)):
            print(f"{mark:4d} {ex:6.2f} {ey:6.2f} {e:5.2f} {eh:6.2f}")
        print(f"\nPosition error (similarity, camera looking straight down): mean {error.mean():.2f}, "
              f"RMS {rms:.2f}, max {error.max():.2f} mm (mark {marks[int(error.argmax())]})")
        print(f"Position error (homography, tilt removed): RMS "
              f"{np.sqrt((np.linalg.norm(homography, axis=1) ** 2).mean()):.2f} mm")
        print(f"Width across flats at the fitted d: {sizes.mean():.2f} +- {sizes.std():.2f} mm "
              f"(nominal 13)")
        if args.debug_dir:
            plot_errors(args.debug_dir / "errors.png", image_size, pixels, residuals, marks, rms)

    extra_images = sorted((args.bench / "extra").glob("*.png"))
    if extra_images:
        print(f"\n{'extra':5s} {'expect':6s} {'valid':5s} {'ok':3s} status  ({'note'})")
        correct = 0
        for index, (path, label) in enumerate(zip(extra_images, labels["extra"]), 1):
            image, result = detect(path, focal_px, distance_mm, params, background)
            save_overlay(args.debug_dir, path, image, result, k)
            ok = result.valid == EXPECT_VALID[label["expect"]]
            correct += ok
            status = result.status if result.valid else failure_reason(result)
            print(f"x{index:<4d} {label['expect']:6s} {str(result.valid):5s} {'yes' if ok else 'NO':3s} "
                  f"{status}  ({label['note']})")
        print(f"Extra: {correct}/{len(extra_images)} as expected")


if __name__ == "__main__":
    main()
