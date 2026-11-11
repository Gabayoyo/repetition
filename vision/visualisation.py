"""Optional OpenCV overlay for the arm tracking loop."""

from __future__ import annotations

import sys

import cv2
import numpy as np

from .arm_chain import ArmChain

WINDOW = "Arm tracking"

ARM_COLOR = (0, 180, 255)
HAND_COLOR = (0, 220, 220)
POINT_COLOR = (120, 120, 120)
TEXT_COLOR = (235, 235, 235)
OPEN_COLOR = (0, 220, 0)
CLOSED_COLOR = (0, 60, 220)

POSE_ARM = ((12, 14), (14, 16))
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def _pixel(landmark, width: int, height: int) -> tuple[int, int] | None:
    if not (0.0 <= landmark.x <= 1.0 and 0.0 <= landmark.y <= 1.0):
        return None
    return int(landmark.x * width), int(landmark.y * height)


class Visualisation:
    """Camera overlay with pose points, hand skeleton and the measured chain."""

    def __init__(self, window: str = WINDOW) -> None:
        self.window = window
        self.enabled = True

    def show(
        self,
        frame: np.ndarray,
        result,
        chain: ArmChain | None,
        quat,
        gripper_open: bool,
    ) -> int:
        if not self.enabled:
            return 255

        canvas = frame.copy()
        height, width = canvas.shape[:2]
        self._draw_pose(canvas, result.pose_landmarks, width, height)
        hand = result.right_hand_landmarks
        self._draw_hand(canvas, hand, width, height)
        self._draw_readout(canvas, chain, quat, gripper_open)

        try:
            cv2.imshow(self.window, canvas)
            return cv2.waitKey(1) & 0xFF
        except cv2.error as exc:
            print(f"visualisation disabled: {exc}", file=sys.stderr)
            self.enabled = False
            self.close()
            return 255

    def _draw_pose(self, canvas, landmarks, width: int, height: int) -> None:
        if not landmarks:
            return
        points = [_pixel(landmark, width, height) for landmark in landmarks]
        for point in points:
            if point is not None:
                cv2.circle(canvas, point, 2, POINT_COLOR, -1, cv2.LINE_AA)
        for first, second in POSE_ARM:
            start, end = points[first], points[second]
            if start is not None and end is not None:
                cv2.line(canvas, start, end, ARM_COLOR, 2, cv2.LINE_AA)
        for index in (12, 14, 16):
            point = points[index]
            if point is not None:
                cv2.circle(canvas, point, 5, ARM_COLOR, -1, cv2.LINE_AA)

    def _draw_hand(self, canvas, landmarks, width: int, height: int) -> None:
        if not landmarks:
            return
        points = [_pixel(landmark, width, height) for landmark in landmarks]
        for first, second in HAND_CONNECTIONS:
            if first >= len(points) or second >= len(points):
                continue
            start, end = points[first], points[second]
            if start is not None and end is not None:
                cv2.line(canvas, start, end, HAND_COLOR, 1, cv2.LINE_AA)
        for point in points:
            if point is not None:
                cv2.circle(canvas, point, 2, HAND_COLOR, -1, cv2.LINE_AA)

    def _draw_readout(
        self,
        canvas,
        chain: ArmChain | None,
        quat,
        gripper_open: bool,
    ) -> None:
        width = canvas.shape[1]
        cv2.rectangle(canvas, (0, 0), (width, 96), (0, 0, 0), -1)
        if chain is None:
            lines = ["NO ARM", "waiting for pose landmarks"]
        else:
            q = np.asarray(quat, dtype=np.float64)
            lines = [
                f"upper {chain.upper:.3f} m   fore {chain.forearm:.3f} m",
                f"quat {q[0]:+.2f} {q[1]:+.2f} {q[2]:+.2f} {q[3]:+.2f}",
            ]
        for row, text in enumerate(lines):
            cv2.putText(
                canvas,
                text,
                (12, 28 + row * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                TEXT_COLOR,
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            canvas,
            "GRIPPER OPEN" if gripper_open else "GRIPPER CLOSED",
            (12, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            OPEN_COLOR if gripper_open else CLOSED_COLOR,
            2,
            cv2.LINE_AA,
        )

    def close(self) -> None:
        try:
            cv2.destroyWindow(self.window)
        except cv2.error:
            pass
