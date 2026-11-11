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


class OneEuroFilter:
    """Scalar One Euro filter with timestamp-driven cutoff adaptation."""

    def __init__(self) -> None:
        self._x: float | None = None
        self._dx: float | None = None
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
            self._x = value
            self._dx = None
            self._last_t = t
            self._last_x = value
            return value

        dt = t - self._last_t
        derivative = (value - self._last_x) / dt
        alpha = _alpha(DERIVATIVE_CUTOFF_HZ, dt)
        self._dx = (
            derivative
            if self._dx is None
            else alpha * derivative + (1.0 - alpha) * self._dx
        )
        cutoff = MIN_CUTOFF_HZ + BETA * abs(self._dx)
        alpha = _alpha(cutoff, dt)
        smoothed = alpha * value + (1.0 - alpha) * self._x
        self._x = smoothed
        self._last_t = t
        self._last_x = value
        return smoothed


class ArmSmoother:
    """Smooths the shoulder/elbow/wrist chain and the hand quaternion."""

    def __init__(self) -> None:
        self._points = [[OneEuroFilter() for _ in range(3)] for _ in range(3)]
        self._quat = [OneEuroFilter() for _ in range(4)]
        self._last_quat: np.ndarray | None = None
        self._last_quat_t: float | None = None

    def update_chain(self, chain: ArmChain, t: float) -> ArmChain:
        shoulder, elbow, wrist = (
            np.array(
                [filter_(value, t) for filter_, value in zip(filters, point)],
                dtype=np.float64,
            )
            for filters, point in zip(
                self._points, (chain.shoulder, chain.elbow, chain.wrist)
            )
        )
        return ArmChain(shoulder=shoulder, elbow=elbow, wrist=wrist)

    def update_quat(self, quaternion, t: float) -> np.ndarray:
        values = np.asarray(quaternion, dtype=np.float64)
        if self._last_quat_t is not None and (
            t <= self._last_quat_t or t - self._last_quat_t > RESET_GAP_S
        ):
            self._last_quat = None
        if self._last_quat is not None and float(np.dot(values, self._last_quat)) < 0.0:
            values = -values
        filtered = np.array(
            [filter_(float(value), t) for filter_, value in zip(self._quat, values)],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(filtered))
        if norm <= 1e-9:
            filtered = IDENTITY_QUAT.copy()
        else:
            filtered /= norm
        self._last_quat = filtered
        self._last_quat_t = t
        return filtered
