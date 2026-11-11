"""Temporal smoothing for the measured arm chain and hand orientation.

MediaPipe's tasks API does not expose the legacy ``smooth_landmarks`` switch, so the
filters here provide the same job explicitly. A One Euro filter keeps its cutoff low while
the arm is still and raises it during fast motion, which limits lag where it matters.
"""

from __future__ import annotations

import math

import numpy as np

from .arm_chain import ArmChain

MIN_CUTOFF_HZ = 1.0
BETA = 0.3
DERIVATIVE_CUTOFF_HZ = 1.0
RESET_GAP_S = 0.5

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


def _alpha(cutoff_hz: float, dt: float) -> float:
    if cutoff_hz <= 0.0:
        return 1.0
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return 1.0 / (1.0 + tau / dt)


class _LowPass:
    __slots__ = ("_value",)

    def __init__(self) -> None:
        self._value: float | None = None

    def filter(self, value: float, alpha: float) -> float:
        if self._value is None:
            self._value = value
        else:
            self._value = alpha * value + (1.0 - alpha) * self._value
        return self._value

    def reset(self) -> None:
        self._value = None


class OneEuroFilter:
    """Scalar One Euro filter with timestamp-driven cutoff adaptation."""

    def __init__(
        self,
        min_cutoff: float = MIN_CUTOFF_HZ,
        beta: float = BETA,
        d_cutoff: float = DERIVATIVE_CUTOFF_HZ,
    ) -> None:
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x = _LowPass()
        self._dx = _LowPass()
        self._last_t: float | None = None
        self._last_x: float | None = None

    def __call__(self, value: float, t: float) -> float:
        value = float(value)
        if (
            self._last_t is None
            or self._last_x is None
            or t <= self._last_t
            or t - self._last_t > RESET_GAP_S
        ):
            self._x.reset()
            self._dx.reset()
            self._x.filter(value, 1.0)
            self._last_t = t
            self._last_x = value
            return value

        dt = t - self._last_t
        derivative = (value - self._last_x) / dt
        smoothed_derivative = self._dx.filter(
            derivative, _alpha(self.d_cutoff, dt)
        )
        cutoff = self.min_cutoff + self.beta * abs(smoothed_derivative)
        smoothed = self._x.filter(value, _alpha(cutoff, dt))
        self._last_t = t
        self._last_x = value
        return smoothed


class VectorFilter:
    """One Euro filter applied component-wise to a fixed-size vector."""

    def __init__(self, size: int, **kwargs) -> None:
        self._filters = [OneEuroFilter(**kwargs) for _ in range(size)]

    def __call__(self, values, t: float) -> np.ndarray:
        return np.array(
            [f(float(v), t) for f, v in zip(self._filters, values)],
            dtype=np.float64,
        )


class QuaternionFilter:
    """Component-wise filtering with sign alignment and renormalisation."""

    def __init__(self, **kwargs) -> None:
        self._filters = [OneEuroFilter(**kwargs) for _ in range(4)]
        self._last: np.ndarray | None = None
        self._last_t: float | None = None

    def __call__(self, quaternion, t: float) -> np.ndarray:
        values = np.asarray(quaternion, dtype=np.float64)
        if self._last_t is not None and (
            t <= self._last_t or t - self._last_t > RESET_GAP_S
        ):
            self._last = None
        if self._last is not None and float(np.dot(values, self._last)) < 0.0:
            values = -values
        filtered = np.array(
            [f(float(v), t) for f, v in zip(self._filters, values)],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(filtered))
        if norm <= 1e-9:
            filtered = IDENTITY_QUAT.copy()
        else:
            filtered /= norm
        self._last = filtered
        self._last_t = t
        return filtered


class ArmSmoother:
    """Smooths the shoulder/elbow/wrist chain and the hand quaternion."""

    def __init__(self, **kwargs) -> None:
        self._points = [VectorFilter(3, **kwargs) for _ in range(3)]
        self._quat = QuaternionFilter(**kwargs)

    def update_chain(self, chain: ArmChain, t: float) -> ArmChain:
        shoulder, elbow, wrist = (
            filter_(point, t)
            for filter_, point in zip(
                self._points, (chain.shoulder, chain.elbow, chain.wrist)
            )
        )
        return ArmChain(shoulder=shoulder, elbow=elbow, wrist=wrist)

    def update_quat(self, quaternion, t: float) -> np.ndarray:
        return self._quat(quaternion, t)
