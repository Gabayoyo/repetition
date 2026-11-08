"""Shared frame representation for camera and tracker loop."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class CaptureFrame:
    """One colour frame"""

    bgr: np.ndarray
    t_s: float
    timestamp_ms: int


class Capture(Protocol):
    """A stream of colour frames"""

    name: str

    def read(self) -> CaptureFrame | None: ...

    def close(self) -> None: ...
