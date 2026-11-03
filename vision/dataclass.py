from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

@dataclass(frozen=True)
class Point:
    group: str
    index: int
    x: float
    y: float
    z: float | None
    visibility: float | None

@dataclass(frozen=True)
class Frame:
    time_s: float
    points: list[Point]

class StereoFrame(NamedTuple):
    image: np.ndarray
    time_s: float
    depth: np.ndarray | None = None
    confidence: np.ndarray | None = None
    depth_time_s: float | None = None
