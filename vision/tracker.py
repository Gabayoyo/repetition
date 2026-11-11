"""MediaPipe Holistic landmark tracking on OAK colour frames."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING
from urllib.request import urlretrieve

# MediaPipe imports matplotlib through its helper stack; keep its cache writable.
if not os.environ.get("MPLCONFIGDIR"):
    os.environ["MPLCONFIGDIR"] = str(Path(tempfile.gettempdir()) / "repetition-mpl")

import cv2
import mediapipe as mp
import numpy as np

if TYPE_CHECKING:
    from .oak import CaptureFrame

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/holistic_landmarker/"
    "holistic_landmarker/float16/1/holistic_landmarker.task"
)
MODELS_DIR = Path(__file__).parent / "models"
MODEL_PATH = MODELS_DIR / "holistic_landmarker.task"


def ensure_model(path: Path = MODEL_PATH, url: str = MODEL_URL) -> Path:
    """Return a local model file, downloading it once if absent."""
    if path.exists():
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    print(f"downloading {url} -> {path}", file=sys.stderr)
    urlretrieve(url, path)
    return path


class HolisticTracker:
    """Runs the tasks HolisticLandmarker in video mode and closes it with the process."""

    def __init__(self, model_path: Path | None = None) -> None:
        options = mp.tasks.vision.HolisticLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(
                model_asset_path=str(model_path or ensure_model())
            ),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            min_pose_detection_confidence=0.5,
            min_pose_landmarks_confidence=0.5,
            min_hand_landmarks_confidence=0.5,
            output_face_blendshapes=False,
            output_segmentation_mask=False,
        )
        self._landmarker = mp.tasks.vision.HolisticLandmarker.create_from_options(
            options
        )

    def process(self, frame: CaptureFrame):
        rgb = np.ascontiguousarray(
            cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2RGB)
        )
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        return self._landmarker.detect_for_video(image, frame.timestamp_ms)

    def close(self) -> None:
        self._landmarker.close()
