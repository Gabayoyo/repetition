from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class ArmChain:
    """Shoulder, elbow and wrist in the robot frame, in metres."""

    shoulder: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray

    @property
    def upper(self) -> float:
        return float(np.linalg.norm(self.elbow - self.shoulder))

    @property
    def forearm(self) -> float:
        return float(np.linalg.norm(self.wrist - self.elbow))