"""Human arm pose from MediaPipe landmarks: frame conversion and hand state.

This module is the perception-to-arm boundary. It returns the measured right-arm chain and
the hand orientation/gripper state; robot-specific scaling and inverse kinematics belong to
the consumer. No calibration, camera, GUI or ROS dependency.

This is the trimmed copy for the Pi-facing repo. Copy it as ``mocap.py`` beside
``arm_chain.py``.
"""

from __future__ import annotations

import math

import numpy as np

from arm_chain import ArmChain

RIGHT_SHOULDER = 12
RIGHT_ELBOW = 14
RIGHT_WRIST = 16

HAND_WRIST = 0
THUMB_TIP = 4
INDEX_FINGER_MCP = 5
INDEX_FINGER_TIP = 8
PINKY_MCP = 17

PINCH_CLOSED = 0.40
PINCH_OPEN = 0.70

MIN_SEGMENT_M = 1e-4
MIN_AXIS_M = 1e-6
IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


def vec3(landmark) -> np.ndarray:
    """A MediaPipe landmark's three coordinates as an array."""
    return np.array([landmark.x, landmark.y, landmark.z], dtype=np.float64)


def to_mujoco(vector) -> np.ndarray:
    """MediaPipe world axes to the mocap frame: x toward the camera, y left, z up."""
    values = np.asarray(vector, dtype=np.float64)
    return np.array([-values[2], values[0], -values[1]], dtype=np.float64)


def _normalised_quat(x: float, y: float, z: float, w: float) -> np.ndarray:
    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    return quat / norm if norm > 1e-9 else IDENTITY_QUAT.copy()


def quat_from_matrix(rotation) -> np.ndarray:
    """Rotation matrix to a normalised (x, y, z, w) quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0.0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        return _normalised_quat(
            (matrix[2, 1] - matrix[1, 2]) * scale,
            (matrix[0, 2] - matrix[2, 0]) * scale,
            (matrix[1, 0] - matrix[0, 1]) * scale,
            0.25 / scale,
        )
    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
        if scale <= 1e-9:
            return IDENTITY_QUAT.copy()
        return _normalised_quat(
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
        )
    if matrix[1, 1] > matrix[2, 2]:
        scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
        if scale <= 1e-9:
            return IDENTITY_QUAT.copy()
        return _normalised_quat(
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
        )
    scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
    if scale <= 1e-9:
        return IDENTITY_QUAT.copy()
    return _normalised_quat(
        (matrix[0, 2] + matrix[2, 0]) / scale,
        (matrix[1, 2] + matrix[2, 1]) / scale,
        0.25 * scale,
        (matrix[1, 0] - matrix[0, 1]) / scale,
    )


def measured_arm(pose_world) -> ArmChain | None:
    """The operator's right arm in the mocap frame, or None when the model missed it."""
    if pose_world is None or len(pose_world) <= RIGHT_WRIST:
        return None
    points = [vec3(pose_world[index]) for index in (RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST)]
    if not all(np.isfinite(point).all() for point in points):
        return None
    shoulder, elbow, wrist = (to_mujoco(point) for point in points)
    chain = ArmChain(shoulder, elbow, wrist)
    if chain.upper <= MIN_SEGMENT_M or chain.forearm <= MIN_SEGMENT_M:
        return None
    return chain


class HandTracker:
    """Wrist orientation and pinch state, held across frames where the hand is missed."""

    def __init__(self) -> None:
        self.quat = IDENTITY_QUAT.copy()
        self.gripper_open = True
        self.pinch_ratio: float | None = None

    def update(self, landmarks) -> bool:
        """Read one frame of hand world landmarks; return whether an orientation was found."""
        if landmarks is None or len(landmarks) <= PINKY_MCP:
            return False
        wrist = vec3(landmarks[HAND_WRIST])
        index = vec3(landmarks[INDEX_FINGER_MCP])
        pinky = vec3(landmarks[PINKY_MCP])
        if not all(np.isfinite(point).all() for point in (wrist, index, pinky)):
            return False
        wrist_robot, index_robot, pinky_robot = (
            to_mujoco(point) for point in (wrist, index, pinky)
        )
        forward = index_robot - wrist_robot
        side = pinky_robot - wrist_robot
        forward_length = float(np.linalg.norm(forward))
        if forward_length <= MIN_AXIS_M:
            return False
        x_axis = forward / forward_length
        z_axis = np.cross(side, forward)
        z_length = float(np.linalg.norm(z_axis))
        if z_length <= MIN_AXIS_M:
            return False
        z_axis = z_axis / z_length
        y_axis = np.cross(z_axis, x_axis)
        self.quat = quat_from_matrix(np.column_stack([x_axis, y_axis, z_axis]))

        palm = float(np.linalg.norm(wrist - index))
        if palm > MIN_SEGMENT_M:
            thumb = vec3(landmarks[THUMB_TIP])
            index_tip = vec3(landmarks[INDEX_FINGER_TIP])
            if np.isfinite(thumb).all() and np.isfinite(index_tip).all():
                self.pinch_ratio = float(np.linalg.norm(thumb - index_tip)) / palm
                if self.pinch_ratio < PINCH_CLOSED:
                    self.gripper_open = False
                elif self.pinch_ratio > PINCH_OPEN:
                    self.gripper_open = True
        return True
