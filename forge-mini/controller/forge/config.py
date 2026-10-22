"""Loads cell.yaml into typed objects shared by the planner, sim and hardware."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

JOINTS = ("base", "shoulder", "elbow", "wrist", "tool")
DEFAULT_CELL = Path(__file__).resolve().parent.parent / "cell.yaml"


@dataclass(frozen=True)
class Links:
    h0: float
    L1: float
    L2: float
    L3: float


@dataclass(frozen=True)
class JointMap:
    offset: float
    sign: float
    min: float
    max: float


@dataclass
class Arm:
    name: str
    role: str
    base: tuple[float, float]
    yaw: float
    tool: str
    board: int
    first_channel: int
    trims: list[float] = field(default_factory=lambda: [0.0] * 5)

    def to_local(self, x: float, y: float) -> tuple[float, float]:
        """World XY -> arm frame (forward = towards the plate centre)."""
        dx, dy = x - self.base[0], y - self.base[1]
        c, s = math.cos(math.radians(-self.yaw)), math.sin(math.radians(-self.yaw))
        return dx * c - dy * s, dx * s + dy * c

    def to_world(self, fx: float, fy: float) -> tuple[float, float]:
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        return self.base[0] + fx * c - fy * s, self.base[1] + fx * s + fy * c


@dataclass
class Cell:
    links: Links
    arms: dict[str, Arm]
    servo_map: dict[str, JointMap]
    tools: dict[str, dict]
    motion: dict
    zone: dict
    locations: dict[str, tuple[float, float, float]]
    park: dict[str, tuple[float, float, float]]
    conveyor: dict
    raw: dict

    def location(self, ref) -> tuple[float, float, float]:
        """Accepts a location name or an [x, y, z] list."""
        if isinstance(ref, str):
            if ref not in self.locations:
                raise KeyError(f"unknown location '{ref}' (known: {', '.join(self.locations)})")
            return self.locations[ref]
        x, y, z = ref
        return float(x), float(y), float(z)

    def joints_to_servos(self, arm: Arm, joints: list[float]) -> list[float]:
        out = []
        for i, name in enumerate(JOINTS):
            m = self.servo_map[name]
            deg = m.offset + m.sign * joints[i] + arm.trims[i]
            out.append(round(min(180.0, max(0.0, deg)), 1))
        return out


def load_cell(path: str | Path = DEFAULT_CELL) -> Cell:
    raw = yaml.safe_load(Path(path).read_text())
    trims = raw.get("trims", {})
    arms = {
        name: Arm(
            name=name,
            role=a["role"],
            base=(float(a["base"][0]), float(a["base"][1])),
            yaw=float(a["yaw"]),
            tool=a["tool"],
            board=int(a["board"]),
            first_channel=int(a["first_channel"]),
            trims=[float(t) for t in trims.get(name, [0] * 5)],
        )
        for name, a in raw["arms"].items()
    }
    return Cell(
        links=Links(**{k: float(v) for k, v in raw["links"].items()}),
        arms=arms,
        servo_map={k: JointMap(**v) for k, v in raw["servo_map"].items()},
        tools=raw["tools"],
        motion=raw["motion"],
        zone=raw["zone"],
        locations={k: tuple(float(c) for c in v) for k, v in raw["locations"].items()},
        park={k: tuple(float(c) for c in v) for k, v in raw["park"].items()},
        conveyor=raw["conveyor"],
        raw=raw,
    )
