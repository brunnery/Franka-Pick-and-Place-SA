"""ChArUco board for the camera-in-flange calibration (hand_eye), as PNG and A4 PDF. No ROS.

  ros2 run sa_tasks charuco_board  # -> data/grid/charuco_<x>x<y>_<square>mm.{png,pdf}

Board from hand_eye.board in tasks.yaml. Print the PDF at 100 % (no "fit to page"), measure one
square with a caliper and enter it as hand_eye.board.square_mm; glue the sheet flat on something
stiff. A ChArUco board is also found when only part of it is in view (the image covers only
about 250 x 140 mm at the observation height).
"""
import cv2
import numpy as np
from PIL import Image

from sa_tasks import config

A4_MM = (210.0, 297.0)
MM_PER_INCH = 25.4
LABEL_SCALE = 1.2


def make_board(board_cfg: dict):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board_cfg["dictionary"]))
    return cv2.aruco.CharucoBoard_create(board_cfg["squares_x"], board_cfg["squares_y"],
                                         board_cfg["square_mm"], board_cfg["marker_mm"],
                                         dictionary)


def main():
    cfg = config.load_tasks()
    b = cfg["hand_eye"]["board"]
    # Whole pixels per square, the resolution follows: a square prints exactly square_mm
    px_per_square = int(round(b["square_mm"] / MM_PER_INCH * b["print_dpi"]))
    dpi = px_per_square * MM_PER_INCH / b["square_mm"]
    board_px = (b["squares_x"] * px_per_square, b["squares_y"] * px_per_square)
    board_image = make_board(b).draw(board_px, marginSize=0, borderBits=1)

    page = np.full((int(round(A4_MM[1] / MM_PER_INCH * dpi)),
                    int(round(A4_MM[0] / MM_PER_INCH * dpi))), 255, np.uint8)
    top, left = [(p - s) // 2 for p, s in zip(page.shape, board_image.shape)]
    if top < 0 or left < 0:
        raise SystemExit(f"board {board_px} px does not fit on A4 at {dpi:.1f} dpi")
    page[top:top + board_image.shape[0], left:left + board_image.shape[1]] = board_image
    label = (f"ChArUco {b['squares_x']}x{b['squares_y']}, square {b['square_mm']:g} mm, marker "
             f"{b['marker_mm']:g} mm, {b['dictionary']} - print at 100 %, measure a square")
    cv2.putText(page, label, (left, top - px_per_square // 6), cv2.FONT_HERSHEY_SIMPLEX,
                LABEL_SCALE * dpi / 300.0, 0, max(1, int(dpi / 150)))

    directory = config.PACKAGE_DIR.parents[2] / "data" / "grid"
    name = f"charuco_{b['squares_x']}x{b['squares_y']}_{b['square_mm']:g}mm"
    cv2.imwrite(str(directory / f"{name}.png"), page)
    Image.fromarray(page).save(directory / f"{name}.pdf", resolution=dpi)
    print(f"{directory / name}.pdf / .png: {board_px[0]} x {board_px[1]} px board at {dpi:.2f} dpi "
          f"({b['squares_x'] * b['square_mm']:g} x {b['squares_y'] * b['square_mm']:g} mm)")


if __name__ == "__main__":
    main()
