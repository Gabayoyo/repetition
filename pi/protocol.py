from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class ArmPacket:
    """The data class that is to be sent over (JSON?)"""
    version: int
    seq: int
    t: float
    valid: bool
    state: str
    shoulder: tuple[float, float, float]
    elbow: tuple[float, float, float]
    wrist: tuple[float, float, float]
    quat: tuple[float, float, float, float]   # x, y, z, w
    gripper_open: bool
    links: tuple[float, float]                # upper, forearm

    def to_dict(self) -> dict:
        return {
            "v": self.version,
            "seq": self.seq,
            "t": self.t,
            "valid": self.valid,
            "state": self.state,
            "shoulder": list(self.shoulder),
            "elbow": list(self.elbow),
            "wrist": list(self.wrist),
            "quat": list(self.quat),
            "gripper": self.gripper_open,
            "links": list(self.links),
        }

    def to_json(self) -> bytes:
        return json.dumps(self.to_dict(), separators=(",", ":")).encode()

    @classmethod
    def from_dict(cls, data: dict) -> "ArmPacket":
        if data.get("v") != 1:
            raise ValueError(f"unsupported packet version: {data.get('v')}")
        # validate lengths, finiteness, quaternion norm, etc.
        return cls(
            version=data["v"],
            seq=int(data["seq"]),
            t=float(data["t"]),
            valid=bool(data["valid"]),
            state=str(data["state"]),
            shoulder=tuple(float(v) for v in data["shoulder"]),
            elbow=tuple(float(v) for v in data["elbow"]),
            wrist=tuple(float(v) for v in data["wrist"]),
            quat=tuple(float(v) for v in data["quat"]),
            gripper_open=bool(data["gripper"]),
            links=tuple(float(v) for v in data["links"]),
        )