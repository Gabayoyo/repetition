# Model artifact resolution and download.

import sys
from pathlib import Path
from urllib.request import urlretrieve

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
