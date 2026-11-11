"""Versioned JSON packet for the measured arm, hand state and link lengths."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

VERSION = 1

Point = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _point(values, name: str) -> Point:
    try:
        coordinates = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain three numbers") from exc
    if len(coordinates) != 3:
        raise ValueError(f"{name} must contain three numbers")
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError(f"{name} contains a non-finite coordinate")
    return coordinates


def _quaternion(values) -> Quaternion:
    try:
        coordinates = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError("quat must contain four numbers") from exc
    if len(coordinates) != 4:
        raise ValueError("quat must contain four numbers")
    norm = math.sqrt(sum(value * value for value in coordinates))
    if norm <= 1e-9 or not math.isfinite(norm):
        raise ValueError("quat has no direction")
    return tuple(value / norm for value in coordinates)


def _links(values) -> tuple[float, float]:
    try:
        lengths = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError("links must contain two numbers") from exc
    if len(lengths) != 2:
        raise ValueError("links must contain two numbers")
    if not all(math.isfinite(value) and value > 0.0 for value in lengths):
        raise ValueError("links must be positive and finite")
    return lengths


@dataclass(frozen=True)
class ArmPacket:
    """One measured arm frame, ready to serialise for the Pi."""

    seq: int
    t: float
    shoulder: Point
    elbow: Point
    wrist: Point
    quat: Quaternion
    gripper_open: bool
    links: tuple[float, float]
    valid: bool = True
    version: int = VERSION

    def __post_init__(self) -> None:
        version = int(self.version)
        if version != VERSION:
            raise ValueError(f"unsupported packet version: {version}")
        seq = int(self.seq)
        if seq < 0:
            raise ValueError("seq must not be negative")
        timestamp = float(self.t)
        if not math.isfinite(timestamp):
            raise ValueError("t must be finite")
        object.__setattr__(self, "version", version)
        object.__setattr__(self, "seq", seq)
        object.__setattr__(self, "t", timestamp)
        object.__setattr__(self, "shoulder", _point(self.shoulder, "shoulder"))
        object.__setattr__(self, "elbow", _point(self.elbow, "elbow"))
        object.__setattr__(self, "wrist", _point(self.wrist, "wrist"))
        object.__setattr__(self, "quat", _quaternion(self.quat))
        object.__setattr__(self, "gripper_open", bool(self.gripper_open))
        object.__setattr__(self, "links", _links(self.links))
        object.__setattr__(self, "valid", bool(self.valid))

    def to_dict(self) -> dict:
        return {
            "v": self.version,
            "seq": self.seq,
            "t": self.t,
            "valid": self.valid,
            "shoulder": list(self.shoulder),
            "elbow": list(self.elbow),
            "wrist": list(self.wrist),
            "quat": list(self.quat),
            "gripper": self.gripper_open,
            "links": list(self.links),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    def to_bytes(self) -> bytes:
        return self.to_json().encode("utf-8")

    @classmethod
    def from_dict(cls, data: dict) -> "ArmPacket":
        try:
            return cls(
                seq=data["seq"],
                t=data["t"],
                shoulder=data["shoulder"],
                elbow=data["elbow"],
                wrist=data["wrist"],
                quat=data["quat"],
                gripper_open=data.get("gripper", data.get("gripper_open", True)),
                links=data["links"],
                valid=data.get("valid", True),
                version=data.get("v", VERSION),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid arm packet: {exc}") from exc

    @classmethod
    def from_json(cls, text) -> "ArmPacket":
        if isinstance(text, bytes):
            text = text.decode("utf-8")
        try:
            data = json.loads(text)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid arm packet JSON: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("arm packet must be a JSON object")
        return cls.from_dict(data)

    @classmethod
    def from_bytes(cls, data: bytes) -> "ArmPacket":
        return cls.from_json(data)
