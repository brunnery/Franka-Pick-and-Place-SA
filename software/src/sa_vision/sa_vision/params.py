"""Read config/vision.yaml outside of ROS (offline tools). No rclpy."""
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import yaml

from sa_vision.detection import NutParams


D_SOURCE_YAML = "yaml"
D_SOURCE_FROM_SIZE = "from_size"
D_SOURCES = (D_SOURCE_YAML, D_SOURCE_FROM_SIZE)
TARGET_NUT = "nut"
TARGET_STUD = "stud"
TARGETS = (TARGET_NUT, TARGET_STUD)


@dataclass(frozen=True)
class VisionConfig:
    nut: NutParams
    d_source: str
    d_from_size_range_mm: Tuple[float, float]
    object_plane_distance_mm: float
    reference_image: str  # path of the empty work area image, "" = local median only

    @property
    def distance_range_mm(self) -> Optional[Tuple[float, float]]:
        """Argument distance_range_mm of detect_nut: the range with from_size, else None."""
        return distance_range(self.d_source, self.d_from_size_range_mm)


def distance_range(d_source: str, d_from_size_range_mm) -> Optional[Tuple[float, float]]:
    if d_source not in D_SOURCES:
        raise ValueError(f"d_source must be one of {D_SOURCES}, not '{d_source}'")
    return tuple(d_from_size_range_mm) if d_source == D_SOURCE_FROM_SIZE else None


def nut_params(values: dict) -> NutParams:
    """NutParams from the "nut" section; lists become tuples."""
    values = dict(values)
    values["roi"] = tuple(values["roi"])
    values["across_flats_range_mm"] = tuple(values["across_flats_range_mm"])
    values["reference_match_range"] = tuple(values["reference_match_range"])
    return NutParams(**values)


def load_vision_config(path: Path) -> VisionConfig:
    params = yaml.safe_load(Path(path).read_text())["vision_node"]["ros__parameters"]
    distance_range(params["d_source"], params["d_from_size_range_mm"])  # unknown d_source fails
    return VisionConfig(nut_params(params["nut"]), params["d_source"],
                        tuple(params["d_from_size_range_mm"]),
                        params["object_plane_distance_mm"], params["reference_image"])
