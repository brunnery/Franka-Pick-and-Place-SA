"""Nut detection on a bright background. Pure NumPy/OpenCV, no ROS.

Pipeline (docs/THEORY.md, Section 2): greyscale -> Gaussian blur -> inverted
Otsu threshold -> opening/closing -> two-level contours (outer boundaries and
holes) -> filters on area, hole ratio, compactness and concentricity -> centre
from image moments.
"""
from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

CENTRE_SOURCES = ("hole", "outer")


@dataclass(frozen=True)
class NutParams:
    """Tunable detection parameters (config/vision.yaml)."""

    blur_kernel_px: int            # Gaussian kernel size, odd
    morph_kernel_px: int           # elliptic structuring element size
    morph_open_iterations: int
    morph_close_iterations: int
    roi: Tuple[int, int, int, int]  # (x, y, width, height) in px, all 0 = full image
    across_flats_mm: float         # nut width across flats s
    hole_diameter_mm: float        # visible diameter of the threaded hole
    area_tolerance: float          # accepted outer area: expected * (1 +- tol)
    hole_ratio_tolerance: float    # accepted hole/outer ratio: expected * (1 +- tol)
    min_compactness: float         # 4 pi A / P^2 of the outer contour
    max_centre_offset_ratio: float  # |hole centre - outer centre| / s_px
    centre_source: str             # "hole" or "outer"


@dataclass
class Candidate:
    """One outer contour and the result of the filters (for debugging)."""

    outer: np.ndarray               # contour in full-image pixel coordinates
    hole: Optional[np.ndarray]      # largest hole of the contour, if any
    centre: Optional[Tuple[float, float]]
    area_px: float
    hole_area_px: float
    compactness: float
    centre_offset_px: float
    rejected: str                   # empty if all filters passed

    @property
    def accepted(self) -> bool:
        return not self.rejected


@dataclass
class NutDetection:
    valid: bool
    status: str
    u: float = math.nan
    v: float = math.nan
    area_px: float = math.nan
    hole_area_px: float = math.nan
    compactness: float = math.nan
    expected_area_px: float = math.nan
    threshold: float = math.nan
    roi: Tuple[int, int, int, int] = (0, 0, 0, 0)
    candidates: List[Candidate] = field(default_factory=list)
    mask: Optional[np.ndarray] = None  # binary foreground of the ROI

    @property
    def num_accepted(self) -> int:
        return sum(c.accepted for c in self.candidates)


def hexagon_area(across_flats: float) -> float:
    """Area of a regular hexagon with width across flats s: sqrt(3)/2 * s^2."""
    return math.sqrt(3.0) / 2.0 * across_flats ** 2


def circle_area(diameter: float) -> float:
    return math.pi / 4.0 * diameter ** 2


def mm_to_px(length_mm: float, focal_px: float, distance_mm: float) -> float:
    """Image length of an object length at distance d (pinhole): l * f / d."""
    return length_mm * focal_px / distance_mm


def expected_area_px(area_mm2: float, focal_px: float, distance_mm: float) -> float:
    """Image area of an object area at distance d: A * (f / d)^2."""
    return area_mm2 * (focal_px / distance_mm) ** 2


def clip_roi(roi: Tuple[int, int, int, int], shape: Tuple[int, ...]) -> Tuple[int, int, int, int]:
    """Clip (x, y, w, h) to the image. A ROI with zero width or height means the full image."""
    height, width = shape[:2]
    x, y, w, h = (int(round(value)) for value in roi)
    if w <= 0 or h <= 0:
        return 0, 0, width, height
    x0, y0 = min(max(x, 0), width), min(max(y, 0), height)
    x1, y1 = min(max(x + w, 0), width), min(max(y + h, 0), height)
    return x0, y0, x1 - x0, y1 - y0


def to_gray(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def segment(gray: np.ndarray, params: NutParams) -> Tuple[np.ndarray, float]:
    """Binary foreground (dark object = 255) and the Otsu threshold."""
    if params.blur_kernel_px > 1:
        gray = cv2.GaussianBlur(gray, (params.blur_kernel_px, params.blur_kernel_px), 0)
    threshold, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    if params.morph_kernel_px > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.morph_kernel_px, params.morph_kernel_px))
        if params.morph_open_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                    iterations=params.morph_open_iterations)
        if params.morph_close_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                    iterations=params.morph_close_iterations)
    return mask, float(threshold)


def contour_centre(contour: np.ndarray) -> Optional[Tuple[float, float]]:
    """Centroid (m10/m00, m01/m00) of the region enclosed by the contour."""
    moments = cv2.moments(contour)
    if moments["m00"] <= 0.0:
        return None
    return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]


def compactness(contour: np.ndarray) -> float:
    """4 pi A / P^2: 1 for a circle, 0.907 for a regular hexagon."""
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0.0:
        return 0.0
    return 4.0 * math.pi * cv2.contourArea(contour) / perimeter ** 2


def _evaluate(outer: np.ndarray, holes: List[np.ndarray], area_range: Tuple[float, float],
              ratio_range: Tuple[float, float], max_offset_px: float,
              params: NutParams) -> Candidate:
    area = cv2.contourArea(outer)
    hole = max(holes, key=cv2.contourArea) if holes else None
    hole_area = cv2.contourArea(hole) if hole is not None else 0.0
    outer_centre = contour_centre(outer)
    hole_centre = contour_centre(hole) if hole is not None else None
    offset = (math.dist(outer_centre, hole_centre)
              if outer_centre is not None and hole_centre is not None else math.nan)
    centre = hole_centre if params.centre_source == "hole" else outer_centre
    comp = compactness(outer)

    if not area_range[0] <= area <= area_range[1]:
        rejected = "area"
    elif hole is None:
        rejected = "no hole"
    elif not ratio_range[0] <= hole_area / area <= ratio_range[1]:
        rejected = "hole ratio"
    elif comp < params.min_compactness:
        rejected = "compactness"
    elif not offset <= max_offset_px:
        rejected = "centre offset"
    elif centre is None:
        rejected = "degenerate"
    else:
        rejected = ""
    return Candidate(outer, hole, centre, area, hole_area, comp, offset, rejected)


def detect_nut(image: np.ndarray, focal_px: float, distance_mm: float,
               params: NutParams) -> NutDetection:
    """Find exactly one nut (dark ring with a bright hole) in the ROI.

    focal_px: focal length in pixels, distance_mm: camera -> nut top face.
    The detection is valid only if exactly one contour passes all filters.
    """
    if params.centre_source not in CENTRE_SOURCES:
        raise ValueError(f"centre_source must be one of {CENTRE_SOURCES}")

    roi = clip_roi(params.roi, image.shape)
    x, y, w, h = roi
    expected_area = expected_area_px(hexagon_area(params.across_flats_mm), focal_px, distance_mm)
    if w == 0 or h == 0:
        return NutDetection(False, "empty roi", expected_area_px=expected_area, roi=roi)

    gray = to_gray(image)[y:y + h, x:x + w]
    mask, threshold = segment(gray, params)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE,
                                           offset=(x, y))

    expected_ratio = circle_area(params.hole_diameter_mm) / hexagon_area(params.across_flats_mm)
    area_range = (expected_area * (1.0 - params.area_tolerance),
                  expected_area * (1.0 + params.area_tolerance))
    ratio_range = (expected_ratio * (1.0 - params.hole_ratio_tolerance),
                   expected_ratio * (1.0 + params.hole_ratio_tolerance))
    max_offset_px = params.max_centre_offset_ratio * mm_to_px(
        params.across_flats_mm, focal_px, distance_mm)

    candidates = []
    if hierarchy is not None:
        # hierarchy[i] = [next, previous, first_child, parent]; with RETR_CCOMP the
        # contours without parent are outer boundaries, their children are holes.
        links = hierarchy[0]
        for i, contour in enumerate(contours):
            if links[i][3] != -1:
                continue
            holes = []
            child = links[i][2]
            while child != -1:
                holes.append(contours[child])
                child = links[child][0]
            candidates.append(_evaluate(contour, holes, area_range, ratio_range,
                                        max_offset_px, params))

    detection = NutDetection(False, "", expected_area_px=expected_area, threshold=threshold,
                             roi=roi, candidates=candidates, mask=mask)
    accepted = [c for c in candidates if c.accepted]
    if len(accepted) != 1:
        detection.status = "no nut found" if not accepted else f"{len(accepted)} nuts found"
        return detection

    best = accepted[0]
    detection.valid = True
    detection.status = "ok"
    detection.u, detection.v = best.centre
    detection.area_px = best.area_px
    detection.hole_area_px = best.hole_area_px
    detection.compactness = best.compactness
    return detection
