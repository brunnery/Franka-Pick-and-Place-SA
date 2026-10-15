"""Read config/vision.yaml outside of ROS (offline tools). No rclpy."""
from dataclasses import dataclass
from pathlib import Path

import yaml

from sa_vision.detection import NutParams


@dataclass(frozen=True)
class VisionConfig:
    nut: NutParams
    object_plane_distance_mm: float
    reference_image: str  # path of the empty work area image, "" = estimate the background


def nut_params(values: dict) -> NutParams:
    """NutParams from the "nut" section; lists become tuples."""
    values = dict(values)
    values["roi"] = tuple(values["roi"])
    values["across_flats_range_mm"] = tuple(values["across_flats_range_mm"])
    return NutParams(**values)


def load_vision_config(path: Path) -> VisionConfig:
    params = yaml.safe_load(Path(path).read_text())["vision_node"]["ros__parameters"]
    return VisionConfig(nut_params(params["nut"]), params["object_plane_distance_mm"],
                        params["reference_image"])
