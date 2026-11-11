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
    version: int = VERSION

    def __post_init__(self) -> None:
        if int(self.version) != VERSION:
            raise ValueError(f"unsupported packet version: {self.version}")

    def to_json(self) -> str:
        return json.dumps(
            {
                "v": self.version,
                "seq": self.seq,
                "t": self.t,
                "shoulder": list(self.shoulder),
                "elbow": list(self.elbow),
                "wrist": list(self.wrist),
                "quat": list(self.quat),
                "gripper": self.gripper_open,
                "links": list(self.links),
            },
            separators=(",", ":"),
        )

    def to_bytes(self) -> bytes:
        return self.to_json().encode("utf-8")

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
        try:
            seq = int(data["seq"])
            if seq < 0:
                raise ValueError("seq must not be negative")
            timestamp = float(data["t"])
            if not math.isfinite(timestamp):
                raise ValueError("t must be finite")
            return cls(
                seq=seq,
                t=timestamp,
                shoulder=_point(data["shoulder"], "shoulder"),
                elbow=_point(data["elbow"], "elbow"),
                wrist=_point(data["wrist"], "wrist"),
                quat=_quaternion(data["quat"]),
                gripper_open=bool(data.get("gripper", data.get("gripper_open", True))),
                links=_links(data["links"]),
                version=int(data.get("v", VERSION)),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid arm packet: {exc}") from exc

    @classmethod
    def from_bytes(cls, data: bytes) -> "ArmPacket":
        return cls.from_json(data)
