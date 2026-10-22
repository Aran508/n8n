"""Forward / inverse kinematics for one 4-DOF arm (base yaw + 3 pitch joints).

Joint conventions (degrees), matching servo_map in cell.yaml:
  base      0 = facing the plate centre, + = counter-clockwise seen from above
  shoulder  angle of the upper arm above horizontal (0..180)
  elbow     forearm relative to upper arm; negative = bends down (elbow-up pose)
  wrist     tool relative to forearm
  lock      tool-changer latch servo, passed through untouched
The tool pitch (phi) is shoulder + elbow + wrist; -90 means pointing straight down.

`l3` is the distance from the wrist axis to the tool centre point (TCP): the
coupler plus whatever tool (and bit) is mounted. It changes with every tool
change, so callers pass the current value.
"""

from __future__ import annotations

import math

from .config import Arm, Cell, JOINTS


class Unreachable(ValueError):
    pass


def forward(cell: Cell, arm: Arm, joints: list[float], l3: float | None = None) -> tuple[float, float, float]:
    """Joint angles -> TCP in world coordinates."""
    L = cell.links
    l3 = L.coupler if l3 is None else l3
    b, t1, t2, t3 = (math.radians(j) for j in joints[:4])
    r = L.L1 * math.cos(t1) + L.L2 * math.cos(t1 + t2) + l3 * math.cos(t1 + t2 + t3)
    z = L.h0 + L.L1 * math.sin(t1) + L.L2 * math.sin(t1 + t2) + l3 * math.sin(t1 + t2 + t3)
    x, y = arm.to_world(r * math.cos(b), r * math.sin(b))
    return x, y, z


def max_reach(cell: Cell) -> float:
    """Largest wrist distance we allow (95% of fully stretched, away from the singularity)."""
    return 0.95 * (cell.links.L1 + cell.links.L2)


def inverse(
    cell: Cell,
    arm: Arm,
    target: tuple[float, float, float],
    pitch: float = -90.0,
    lock: float = 0.0,
    l3: float | None = None,
) -> list[float]:
    """World TCP target + tool pitch -> [base, shoulder, elbow, wrist, lock] joint angles.

    Raises Unreachable with a human-readable reason (used by the planner and by
    the AI agent to correct its own recipes).
    """
    L = cell.links
    l3 = L.coupler if l3 is None else l3
    x, y, z = target
    fx, fy = arm.to_local(x, y)
    base = math.degrees(math.atan2(fy, fx))
    r = math.hypot(fx, fy)

    phi = math.radians(pitch)
    rw = r - l3 * math.cos(phi)
    zw = z - l3 * math.sin(phi) - L.h0
    d = math.hypot(rw, zw)
    if d > max_reach(cell):
        raise Unreachable(
            f"{arm.name} cannot reach ({x:.0f}, {y:.0f}, {z:.0f}): wrist needs {d:.0f} mm, "
            f"max is {max_reach(cell):.0f} mm"
        )
    if d < abs(L.L1 - L.L2) + 10:
        raise Unreachable(f"{arm.name}: ({x:.0f}, {y:.0f}, {z:.0f}) is too close to the base")

    c2 = (d * d - L.L1**2 - L.L2**2) / (2 * L.L1 * L.L2)
    elbow = -math.acos(max(-1.0, min(1.0, c2)))
    shoulder = math.atan2(zw, rw) + math.atan2(L.L2 * math.sin(-elbow), L.L1 + L.L2 * math.cos(elbow))
    wrist = phi - shoulder - elbow

    joints = [base, math.degrees(shoulder), math.degrees(elbow), math.degrees(wrist), lock]
    for name, value in zip(JOINTS[:4], joints[:4]):
        m = cell.servo_map[name]
        if not (m.min - 1e-6 <= value <= m.max + 1e-6):
            raise Unreachable(
                f"{arm.name}: ({x:.0f}, {y:.0f}, {z:.0f}) needs {name}={value:.0f} deg, "
                f"limit is {m.min:.0f}..{m.max:.0f}"
            )
    return [round(j, 2) for j in joints]
