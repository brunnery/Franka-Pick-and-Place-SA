"""Nut detection on a bright table, independent of the nut size. Pure NumPy/OpenCV, no ROS.

Pipeline (docs/THEORY.md, Section 2): greyscale -> Gaussian blur -> divide by the
reference image of the empty work area, if there is one -> divide by a local median
(table brightness near the nut) -> foreground: either a two-sided threshold (bare
metal is brighter or clearly darker than the table) or the gradient magnitude of
I/B (sharp metal edges, not soft shadows) -> closing/opening -> outer contours -> convex hull as
silhouette -> filters on metric size, width ratio s/e, dark core (thread hole) and
concentricity -> centre from the silhouette moments, width across flats in mm and
the thread size if it is unambiguous.
"""
from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

# Width across flats s [mm] of hex nuts, ISO 4032 and DIN 934 (they differ for M10 and M12)
STANDARD_ACROSS_FLATS = (
    (7.0, "M4"), (8.0, "M5"), (10.0, "M6"), (13.0, "M8"),
    (16.0, "M10"), (17.0, "M10"), (18.0, "M12"), (19.0, "M12"),
)
BACKGROUND_REFERENCE = "reference"
BACKGROUND_ESTIMATED = "estimated"
SEGMENTATION_THRESHOLD = "threshold"
SEGMENTATION_EDGES = "edges"
SEGMENTATIONS = (SEGMENTATION_THRESHOLD, SEGMENTATION_EDGES)
# Filters in the order they are applied, and how a rejection is shown to the user
FILTER_ORDER = ("size", "width ratio", "degenerate", "no hole", "hole ratio", "centre offset")
REJECTION_TEXT = {
    "size": "wrong size", "width ratio": "not hexagonal (s/e)", "degenerate": "degenerate",
    "no hole": "no hole", "hole ratio": "hole size", "centre offset": "hole off-centre",
}
# Grey value of the bare table after dividing by the reference image (headroom for highlights)
REFERENCE_GREY = 128.0


@dataclass(frozen=True)
class NutParams:
    """Tunable detection parameters (config/vision.yaml)."""

    blur_kernel_px: int              # Gaussian kernel size, odd
    morph_kernel_px: int             # elliptic structuring element size
    morph_close_iterations: int      # joins highlights and dark facets of one nut
    morph_open_iterations: int       # removes specks
    roi: Tuple[int, int, int, int]   # (x, y, width, height) in px, all 0 = full image
    segmentation: str                # "threshold" (I/B level) or "edges" (gradient of I/B)
    edge_threshold: float            # edges: foreground if |Sobel 3x3 of I/B| > this
    background_downscale: int        # local median is computed at 1/this resolution
    background_window_ratio: float   # median window / largest accepted s in px
    brighter_threshold: float        # foreground if I/B > 1 + this
    darker_threshold: float          # foreground if I/B < 1 - this
    core_threshold: float            # dark core (thread hole) if I/B < 1 - this
    across_flats_range_mm: Tuple[float, float]  # smallest and largest nut, s in mm
    target_across_flats_mm: float    # accept only this s (mm); 0 = any s within the range
    size_tolerance: float            # accepted s: target (or range) * (1 +- tol)
    size_class_tolerance: float      # thread size only if all standard s within +-tol agree
    min_width_ratio: float           # s/e of the silhouette: hexagon 0.866, square 0.707
    max_width_ratio: float           # circle 1.0
    min_hole_ratio: float            # dark core area / silhouette area
    max_hole_ratio: float
    max_centre_offset_ratio: float   # |core centre - silhouette centre| / s_px


@dataclass
class Candidate:
    """One silhouette and the result of the filters (for debugging)."""

    outer: np.ndarray                # convex hull in full-image pixel coordinates
    hole: Optional[np.ndarray]       # dark core, if it was evaluated and found
    centre: Optional[Tuple[float, float]]
    area_px: float
    hole_area_px: float
    across_flats_px: float           # s: minimum width of the silhouette
    across_corners_px: float         # e: maximum width (diameter) of the silhouette
    distance_mm: float               # d used for this candidate (given, or f * s_target / s_px)
    across_flats_mm: float
    compactness: float
    centre_offset_px: float
    size: str                        # thread size, empty if not unambiguous
    rejected: str                    # empty if all filters passed

    @property
    def accepted(self) -> bool:
        return not self.rejected

    @property
    def width_ratio(self) -> float:
        return self.across_flats_px / self.across_corners_px if self.across_corners_px else 0.0

    @property
    def hole_ratio(self) -> float:
        return self.hole_area_px / self.area_px if self.area_px else 0.0


@dataclass
class NutDetection:
    valid: bool
    status: str
    u: float = math.nan
    v: float = math.nan
    across_flats_mm: float = math.nan
    across_flats_px: float = math.nan
    size: str = ""
    distance_mm: float = math.nan    # d used for the result (given, or estimated from the size)
    distance_from_size: bool = False  # True: d = f * s_target / s_px, s is assumed, not measured
    area_px: float = math.nan
    hole_area_px: float = math.nan
    width_ratio: float = math.nan
    compactness: float = math.nan
    background: str = ""
    roi: Tuple[int, int, int, int] = (0, 0, 0, 0)
    candidates: List[Candidate] = field(default_factory=list)
    mask: Optional[np.ndarray] = None  # binary foreground of the ROI (after morphology)
    ratio: Optional[np.ndarray] = None  # I/B of the ROI (debug view)

    @property
    def num_accepted(self) -> int:
        return sum(c.accepted for c in self.candidates)


def hexagon_area(across_flats: float) -> float:
    """Area of a regular hexagon with width across flats s: sqrt(3)/2 * s^2."""
    return math.sqrt(3.0) / 2.0 * across_flats ** 2


def mm_to_px(length_mm: float, focal_px: float, distance_mm: float) -> float:
    """Image length of an object length at distance d (pinhole): l * f / d."""
    return length_mm * focal_px / distance_mm


def px_to_mm(length_px: float, focal_px: float, distance_mm: float) -> float:
    """Object length at distance d from its image length: l_px * d / f."""
    return length_px * distance_mm / focal_px


def classify_size(across_flats_mm: float, tolerance: float) -> str:
    """Thread size whose standard s lies within +-tolerance (relative), if it is unique.

    Returns "" when no standard size or several thread sizes match (e.g. 17.5 mm
    lies between M10 DIN 934 and M12 ISO 4032).
    """
    sizes = {size for s, size in STANDARD_ACROSS_FLATS
             if abs(across_flats_mm - s) <= tolerance * s}
    return sizes.pop() if len(sizes) == 1 else ""


def accepted_size_range(params: NutParams) -> Tuple[float, float]:
    """Window for s in mm: around the target, or around the configured range."""
    low, high = params.across_flats_range_mm
    if params.target_across_flats_mm > 0.0:
        low = high = params.target_across_flats_mm
    return low * (1.0 - params.size_tolerance), high * (1.0 + params.size_tolerance)


def distance_from_size(across_flats_px: float, across_flats_mm: float,
                       focal_px: float) -> float:
    """Distance at which a nut of known s appears with s_px: d = f * s / s_px."""
    return focal_px * across_flats_mm / across_flats_px if across_flats_px > 0 else math.inf


def accepted_size_range_px(params: NutParams, focal_px: float, distance_mm: Optional[float],
                           distance_range_mm: Optional[Tuple[float, float]]
                           ) -> Tuple[float, float]:
    """Window for s_px.

    Known d: the mm window of accepted_size_range at d. d estimated from the size:
    the target window seen from anywhere in (d_min, d_max), i.e. only the plausibility
    of the implied d is checked (the size itself cannot be measured then).
    """
    low_mm, high_mm = accepted_size_range(params)
    if distance_range_mm is not None:
        d_min, d_max = distance_range_mm
        return mm_to_px(low_mm, focal_px, d_max), mm_to_px(high_mm, focal_px, d_min)
    return mm_to_px(low_mm, focal_px, distance_mm), mm_to_px(high_mm, focal_px, distance_mm)


def failure_reason(detection: NutDetection) -> str:
    """Short reason why the detection is invalid, for the debug image ("" if valid).

    Several accepted candidates: their number. Otherwise the candidate that passed
    the most filters (larger area on a tie) and the filter that stopped it.
    """
    if detection.valid:
        return ""
    if detection.num_accepted > 1:
        return f"{detection.num_accepted} candidates"
    if not detection.candidates:
        return "nothing in view" if detection.status == "no nut found" else detection.status
    best = max(detection.candidates,
               key=lambda c: (FILTER_ORDER.index(c.rejected), c.area_px))
    text = REJECTION_TEXT[best.rejected]
    if best.rejected == "size":
        if detection.distance_from_size:
            text += f" (d would be {best.distance_mm:.0f} mm)"
        else:
            text += f" (s = {best.across_flats_mm:.1f} mm)"
    return text


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


def _blur(gray: np.ndarray, params: NutParams) -> np.ndarray:
    if params.blur_kernel_px > 1:
        return cv2.GaussianBlur(gray, (params.blur_kernel_px, params.blur_kernel_px), 0)
    return gray


def _odd(value: float) -> int:
    n = max(int(math.ceil(value)), 1)
    return n if n % 2 else n + 1


def estimate_background(gray: np.ndarray, window_px: float, downscale: int) -> np.ndarray:
    """Local table brightness: median filter over a window somewhat larger than the nut.

    The median ignores objects that cover less than half of the window. Near the
    nut the window also contains its soft shadow, which pulls the estimate towards
    the shadow level and so suppresses the shadow in I/B. Computed on a downscaled
    image for speed and interpolated back.
    """
    height, width = gray.shape
    downscale = max(downscale, 1)
    small = cv2.resize(gray, (max(width // downscale, 1), max(height // downscale, 1)),
                       interpolation=cv2.INTER_AREA)
    kernel = _odd(window_px / downscale)
    largest = _odd(min(small.shape)) - 2  # largest odd kernel that fits into the image
    kernel = min(kernel, largest)
    if kernel >= 3:
        small = cv2.medianBlur(small, kernel)
    return cv2.resize(small, (width, height), interpolation=cv2.INTER_LINEAR)


def ratio_image(gray: np.ndarray, background: np.ndarray) -> np.ndarray:
    """I / B: 1 on the bare table, independent of the illumination level."""
    return gray.astype(np.float32) / np.maximum(background.astype(np.float32), 1.0)


def threshold(ratio: np.ndarray, params: NutParams) -> np.ndarray:
    """Binary foreground (255) before morphology: clearly brighter or darker than the table."""
    return ((ratio > 1.0 + params.brighter_threshold)
            | (ratio < 1.0 - params.darker_threshold)).astype(np.uint8) * 255


def edges(ratio: np.ndarray, params: NutParams) -> np.ndarray:
    """Binary foreground (255) before morphology: strong gradient of I/B.

    Metal edges (outline, facets, chamfer, hole) are sharp; the soft shadow of the
    nut and a slowly varying table brightness have a small gradient and drop out.
    """
    gx = cv2.Sobel(ratio, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(ratio, cv2.CV_32F, 0, 1, ksize=3)
    return (np.hypot(gx, gy) > params.edge_threshold).astype(np.uint8) * 255


def fill_holes(mask: np.ndarray) -> np.ndarray:
    """Fill the inside of every outer contour (an edge outline becomes a region)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, contours, -1, 255, -1)
    return filled


def segment(ratio: np.ndarray, params: NutParams) -> np.ndarray:
    """Foreground after closing (joins the parts of one nut) and opening (removes specks).

    Edges: the closed outline is filled before the opening, otherwise a thin edge
    ring of a nut with little texture would be removed as a speck.
    """
    if params.segmentation not in SEGMENTATIONS:
        raise ValueError(f"segmentation must be one of {SEGMENTATIONS}, "
                         f"not '{params.segmentation}'")
    if params.segmentation == SEGMENTATION_EDGES:
        mask = edges(ratio, params)
    else:
        mask = threshold(ratio, params)
    if params.morph_kernel_px > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.morph_kernel_px, params.morph_kernel_px))
        if params.morph_close_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                    iterations=params.morph_close_iterations)
        if params.segmentation == SEGMENTATION_EDGES:
            mask = fill_holes(mask)
        if params.morph_open_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                    iterations=params.morph_open_iterations)
    return mask


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


def caliper_widths(hull: np.ndarray) -> Tuple[float, float]:
    """Minimum and maximum width of a convex polygon (rotating calipers).

    The minimum width is attained with one caliper flush with an edge; for a hexagon
    it is the width across flats s. The maximum width is the diameter (across
    corners e for a hexagon).
    """
    points = hull.reshape(-1, 2).astype(float)
    if len(points) < 3:
        return 0.0, 0.0
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    edges, lengths = edges[lengths > 0], lengths[lengths > 0]
    normals = np.stack([-edges[:, 1], edges[:, 0]], axis=1) / lengths[:, None]
    projections = points @ normals.T
    min_width = float((projections.max(axis=0) - projections.min(axis=0)).min())
    differences = points[:, None, :] - points[None, :, :]
    max_width = float(np.sqrt((differences ** 2).sum(axis=2).max()))
    return min_width, max_width


def _dark_core(hull: np.ndarray, ratio: np.ndarray, offset: Tuple[int, int],
               threshold: float) -> Optional[np.ndarray]:
    """Largest region inside the silhouette that is clearly darker than the table."""
    x, y, w, h = cv2.boundingRect(hull)
    local = hull - np.array([x, y])
    inside = np.zeros((h, w), np.uint8)
    cv2.drawContours(inside, [local], -1, 255, -1)
    x0, y0 = x - offset[0], y - offset[1]
    window = ratio[y0:y0 + h, x0:x0 + w]
    core = ((window < 1.0 - threshold) & (inside > 0)).astype(np.uint8) * 255
    contours, _ = cv2.findContours(core, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE,
                                   offset=(x, y))
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def _evaluate(contour: np.ndarray, ratio: np.ndarray, offset: Tuple[int, int],
              focal_px: float, distance_mm: Optional[float], size_range_px: Tuple[float, float],
              params: NutParams) -> Candidate:
    """distance_mm None: d is estimated from s_px and the target size (s is then assumed)."""
    hull = cv2.convexHull(contour)
    area = cv2.contourArea(hull)
    s_px, e_px = caliper_widths(hull)
    if distance_mm is None:
        distance_mm = distance_from_size(s_px, params.target_across_flats_mm, focal_px)
        s_mm, size = params.target_across_flats_mm, ""
    else:
        s_mm = px_to_mm(s_px, focal_px, distance_mm)
        size = classify_size(s_mm, params.size_class_tolerance)
    centre = contour_centre(hull)
    candidate = Candidate(hull, None, centre, area, 0.0, s_px, e_px, distance_mm, s_mm,
                          compactness(hull), math.nan, size, "")

    if not size_range_px[0] <= s_px <= size_range_px[1]:
        candidate.rejected = "size"
    elif not params.min_width_ratio <= candidate.width_ratio <= params.max_width_ratio:
        candidate.rejected = "width ratio"
    elif centre is None:
        candidate.rejected = "degenerate"
    if candidate.rejected:
        return candidate

    hole = _dark_core(hull, ratio, offset, params.core_threshold)
    hole_centre = contour_centre(hole) if hole is not None else None
    if hole_centre is None:
        candidate.rejected = "no hole"
        return candidate
    candidate.hole = hole
    candidate.hole_area_px = cv2.contourArea(hole)
    candidate.centre_offset_px = math.dist(centre, hole_centre)
    if not params.min_hole_ratio <= candidate.hole_ratio <= params.max_hole_ratio:
        candidate.rejected = "hole ratio"
    elif candidate.centre_offset_px > params.max_centre_offset_ratio * s_px:
        candidate.rejected = "centre offset"
    return candidate


def detect_nut(image: np.ndarray, focal_px: float, distance_mm: Optional[float],
               params: NutParams, background: Optional[np.ndarray] = None,
               distance_range_mm: Optional[Tuple[float, float]] = None) -> NutDetection:
    """Find exactly one nut in the ROI.

    focal_px: focal length in pixels, distance_mm: camera -> nut top face.
    background: image of the empty work area from the same pose and exposure
    (method A'); None uses the local median of the image alone (A).
    distance_range_mm: (d_min, d_max) estimates d from the size instead
    (d = f * target / s_px, hand-held tests); distance_mm is then ignored. The nut
    size is assumed in this mode, and the size filter only checks that d lies in
    the range.
    The detection is valid only if exactly one silhouette passes all filters.
    """
    if background is not None and background.shape[:2] != image.shape[:2]:
        raise ValueError(f"reference image {background.shape[:2]} does not match the "
                         f"camera image {image.shape[:2]}")
    from_size = distance_range_mm is not None
    if from_size and params.target_across_flats_mm <= 0.0:
        raise ValueError("estimating d from the size needs target_across_flats_mm > 0")

    roi = clip_roi(params.roi, image.shape)
    x, y, w, h = roi
    mode = BACKGROUND_ESTIMATED if background is None else BACKGROUND_REFERENCE
    if w == 0 or h == 0:
        return NutDetection(False, "empty roi", background=mode, roi=roi)

    size_range_px = accepted_size_range_px(params, focal_px, distance_mm, distance_range_mm)
    gray = _blur(to_gray(image)[y:y + h, x:x + w], params)
    if background is not None:
        # Divide out static objects, gradients and the table texture first
        reference = _blur(to_gray(background)[y:y + h, x:x + w], params)
        gray = np.clip(ratio_image(gray, reference) * REFERENCE_GREY, 0, 255).astype(np.uint8)
    # Local median at a scale near the nut size: also absorbs most of its soft shadow
    window_px = params.background_window_ratio * size_range_px[1]
    table = estimate_background(gray, window_px, params.background_downscale)
    ratio = ratio_image(gray, table)
    mask = segment(ratio, params)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE,
                                   offset=(x, y))
    candidate_distance = None if from_size else distance_mm
    candidates = [_evaluate(c, ratio, (x, y), focal_px, candidate_distance, size_range_px,
                            params) for c in contours]

    detection = NutDetection(False, "", background=mode, roi=roi, candidates=candidates,
                             mask=mask, ratio=ratio, distance_from_size=from_size)
    if not from_size:
        detection.distance_mm = distance_mm
    accepted = [c for c in candidates if c.accepted]
    if len(accepted) != 1:
        if not accepted:
            detection.status = "no nut found"
        else:
            if from_size:
                values = ", ".join(f"{c.distance_mm:.0f}" for c in accepted)
                detection.status = f"{len(accepted)} nuts found (d = {values} mm)"
            else:
                values = ", ".join(f"{c.across_flats_mm:.1f}" for c in accepted)
                detection.status = f"{len(accepted)} nuts found (s = {values} mm)"
        return detection

    best = accepted[0]
    detection.valid = True
    detection.status = "ok"
    detection.u, detection.v = best.centre
    detection.across_flats_mm = best.across_flats_mm
    detection.across_flats_px = best.across_flats_px
    detection.size = best.size
    detection.distance_mm = best.distance_mm
    detection.area_px = best.area_px
    detection.hole_area_px = best.hole_area_px
    detection.width_ratio = best.width_ratio
    detection.compactness = best.compactness
    return detection
