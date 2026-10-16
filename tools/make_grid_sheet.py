"""Printable sheet with a grid of nut positions for bench tests of the detection.

  python3 tools/make_grid_sheet.py [--cols 7] [--rows 4] [--pitch-mm 30] [--out data/grid]

Writes grid_sheet.pdf (A4 landscape, print at 100 % / "actual size") and grid.yaml
(the layout, read by the evaluation). Marks are numbered row by row, starting top
left as seen in the camera image; the side labelled "IMAGE TOP" goes to the top of
the camera image. Check the 100 mm scale bar with a ruler after printing.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

PAGE_MM = (297.0, 210.0)       # A4 landscape
MM_PER_INCH = 25.4
MARK_ARM_MM = 3.0              # half length of a cross, hidden under an M8 nut (s = 13 mm)
MARK_LINE_PT = 0.4
MARK_GREY = "0.55"
LABEL_OFFSET_MM = (8.0, -8.0)  # label position relative to its mark
LABEL_SIZE_PT = 6
CENTRE_RADIUS_MM = 2.0
SCALE_BAR_MM = 100.0
MARGIN_MM = 12.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cols", type=int, default=7)
    parser.add_argument("--rows", type=int, default=4)
    parser.add_argument("--pitch-mm", type=float, default=30.0)
    parser.add_argument("--out", type=Path, default=Path("data/grid"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    width, height = PAGE_MM
    fig = plt.figure(figsize=(width / MM_PER_INCH, height / MM_PER_INCH))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)  # y up on the page, so the top row has the largest y
    ax.axis("off")

    cx, cy = width / 2, height / 2
    marks = []
    for row in range(args.rows):
        for col in range(args.cols):
            # Sheet coordinates in mm relative to the grid centre: x right, y down (as in the image)
            x = (col - (args.cols - 1) / 2) * args.pitch_mm
            y = (row - (args.rows - 1) / 2) * args.pitch_mm
            number = len(marks) + 1
            marks.append({"id": number, "x_mm": x, "y_mm": y})
            px, py = cx + x, cy - y
            ax.plot([px - MARK_ARM_MM, px + MARK_ARM_MM], [py, py], color=MARK_GREY,
                    lw=MARK_LINE_PT)
            ax.plot([px, px], [py - MARK_ARM_MM, py + MARK_ARM_MM], color=MARK_GREY,
                    lw=MARK_LINE_PT)
            ax.text(px + LABEL_OFFSET_MM[0], py + LABEL_OFFSET_MM[1], str(number),
                    color=MARK_GREY, fontsize=LABEL_SIZE_PT, ha="center", va="center")

    # Grid centre: place it under the principal point (cross in the capture preview)
    ax.add_patch(plt.Circle((cx, cy), CENTRE_RADIUS_MM, fill=False, color=MARK_GREY,
                            lw=MARK_LINE_PT))

    # Orientation and scale, outside the grid
    ax.text(cx, height - MARGIN_MM, "IMAGE TOP", ha="center", va="top", fontsize=10)
    bar_y = MARGIN_MM
    ax.plot([cx - SCALE_BAR_MM / 2, cx + SCALE_BAR_MM / 2], [bar_y, bar_y], color="k", lw=1.0)
    for end in (-1, 1):
        x_end = cx + end * SCALE_BAR_MM / 2
        ax.plot([x_end, x_end], [bar_y - 2, bar_y + 2], color="k", lw=1.0)
    ax.text(cx, bar_y + 3, f"{SCALE_BAR_MM:.0f} mm (check after printing)", ha="center",
            va="bottom", fontsize=7)
    ax.text(MARGIN_MM, height - MARGIN_MM,
            f"{args.cols} x {args.rows} marks, pitch {args.pitch_mm:g} mm. Circle = grid "
            f"centre; place it under the principal point.",
            fontsize=6, va="top")

    fig.savefig(args.out / "grid_sheet.pdf")
    layout = {"cols": args.cols, "rows": args.rows, "pitch_mm": args.pitch_mm,
              "frame": "x right, y down as in the camera image, origin = grid centre",
              "marks": marks}
    (args.out / "grid.yaml").write_text(yaml.safe_dump(layout, sort_keys=False))
    print(f"Wrote {args.out / 'grid_sheet.pdf'} and {args.out / 'grid.yaml'}")


if __name__ == "__main__":
    main()
