"""Reach, static torque and cutting-force check for every tool (`forge budget`).

Worst case for every pitch joint is the arm stretched out horizontally, so the
torque each servo must hold is sum(mass * horizontal lever arm) of everything
outboard of it. We compare that against stall torque and want to stay under
50% (hobby servos overheat and jitter above that).

For the spindle we also ask: how hard can the arm push down at the fixture
before a joint passes 50%? That is the feed force available for reaming.
"""

from __future__ import annotations

import math

from .config import Cell
from .kinematics import inverse

# Stall torque at 6 V (kg*cm) and mass (g) from typical datasheets.
SERVOS = {
    "MG996R": {"stall": 11.0, "mass": 55.0},
    "MG90S": {"stall": 2.2, "mass": 13.4},
    "SG90": {"stall": 1.8, "mass": 9.0},
}

DESIGN = {"base": "MG996R", "shoulder": "MG996R", "elbow": "MG996R", "wrist": "MG90S", "lock": "SG90"}
LINK_MASS_G = {"upper": 8.0, "fore": 7.0, "coupler": 10.0}  # printed PLA + coupler magnets
PAYLOAD_G = {"gripper": 30.0, "vacuum": 30.0}  # only part-carrying tools carry a part
SAFE_FRACTION = 0.5
G = 9.81


def torque_table(cell: Cell, tool_mass: float = 22.0, tool_len: float = 38.0, payload_g: float = 30.0) -> list[dict]:
    L = cell.links
    elbow_servo = SERVOS[DESIGN["elbow"]]["mass"]
    wrist_servo = SERVOS[DESIGN["wrist"]]["mass"]
    lock_servo = SERVOS[DESIGN["lock"]]["mass"]
    reach = L.L1 + L.L2

    # (mass g, distance from the shoulder axis in mm)
    loads = [
        (LINK_MASS_G["upper"], L.L1 / 2),
        (elbow_servo, L.L1),
        (LINK_MASS_G["fore"], L.L1 + L.L2 / 2),
        (wrist_servo, reach),
        (LINK_MASS_G["coupler"] + lock_servo, reach + L.coupler / 2),
        (tool_mass, reach + L.coupler + tool_len / 2),
        (payload_g, reach + L.coupler + tool_len),
    ]
    pivots = {"shoulder": 0.0, "elbow": L.L1, "wrist": reach}

    rows = []
    for joint, pivot in pivots.items():
        need = sum(m * (d - pivot) / 10 for m, d in loads if d > pivot) / 1000  # kg*cm
        servo = DESIGN[joint]
        stall = SERVOS[servo]["stall"]
        rows.append(
            {
                "joint": joint,
                "servo": servo,
                "torque_kgcm": round(need, 2),
                "stall_kgcm": stall,
                "load_pct": round(100 * need / stall),
                "ok": need <= SAFE_FRACTION * stall,
            }
        )
    return rows


def worst_by_tool(cell: Cell) -> list[dict]:
    out = []
    for t in cell.tools.values():
        length = t.length + (max(b.stickout for b in cell.bits.values()) if t.type == "spindle" else 0)
        rows = torque_table(cell, t.mass, length, PAYLOAD_G.get(t.type, 0.0))
        worst = max(rows, key=lambda r: r["load_pct"])
        out.append({"tool": t.name, "type": t.type, "mass": t.mass, "worst_joint": worst["joint"],
                    "load_pct": worst["load_pct"], "ok": all(r["ok"] for r in rows)})
    return out


def feed_force(cell: Cell, arm_name: str = "arm4", at: str = "fixture.hole1") -> dict:
    """Max downward push at a location before any pitch joint passes 50% of stall.

    A vertical force F at the TCP needs F * (horizontal distance joint->TCP) of
    torque at each pitch joint; the arm's own weight is already using some.
    """
    arm = cell.arms[arm_name]
    spindle = cell.tools["spindle"]
    bit = max(cell.bits.values(), key=lambda b: b.stickout)
    l3 = cell.links.coupler + spindle.length + bit.stickout
    j = inverse(cell, arm, cell.location(at), cell.motion["tool_pitch"], 0, l3)
    L = cell.links
    t1, t2 = math.radians(j[1]), math.radians(j[1] + j[2])
    elbow_r = L.L1 * math.cos(t1)
    wrist_r = elbow_r + L.L2 * math.cos(t2)
    tcp_r = wrist_r  # tool hangs straight down from the wrist
    static = {r["joint"]: r["torque_kgcm"] for r in torque_table(cell, spindle.mass, spindle.length + bit.stickout, 0)}
    limits = {}
    for joint, lever_mm in (("shoulder", tcp_r), ("elbow", tcp_r - elbow_r)):
        spare = SAFE_FRACTION * SERVOS[DESIGN[joint]]["stall"] - static[joint]
        lever_cm = max(abs(lever_mm) / 10, 0.1)
        limits[joint] = round(max(spare, 0) / lever_cm * G, 1)  # kg*cm / cm -> kg -> N
    joint = min(limits, key=limits.get)
    return {"at": at, "limits_N": limits, "max_feed_N": limits[joint], "limited_by": joint}


def reach_summary(cell: Cell) -> dict:
    L = cell.links
    return {
        "max_wrist_mm": round(0.95 * (L.L1 + L.L2)),
        "arm_to_plate_centre_mm": 100,
        "coupler_mm": L.coupler,
    }


def report(cell: Cell) -> str:
    lines = ["Reach", "-----"]
    for k, v in reach_summary(cell).items():
        lines.append(f"  {k:<26}{v}")
    lines += ["", "Static torque, arm stretched flat (gripper + 30 g part)", "-" * 56]
    lines.append(f"  {'joint':<10}{'servo':<8}{'needs':>8}{'stall':>8}{'load':>7}")
    for r in torque_table(cell):
        flag = "ok" if r["ok"] else "TOO HIGH"
        lines.append(
            f"  {r['joint']:<10}{r['servo']:<8}{r['torque_kgcm']:>6.2f}kg{r['stall_kgcm']:>6.1f}kg"
            f"{r['load_pct']:>6}%  {flag}"
        )
    lines += ["", "Worst joint for each tool", "-" * 25]
    for r in worst_by_tool(cell):
        lines.append(f"  {r['tool']:<10}{r['mass']:>5.0f} g  {r['worst_joint']:<9}{r['load_pct']:>4}%  {'ok' if r['ok'] else 'TOO HIGH'}")
    f = feed_force(cell)
    lines += ["", "Feed force for reaming at the fixture", "-" * 37,
              f"  max push {f['max_feed_N']} N (limited by the {f['limited_by']})",
              "  enough for reaming printed pilot holes and light drilling in plastic or wood;",
              "  not for metal: hobby servos flex, so keep cuts light and let the reamer follow the pilot hole"]
    return "\n".join(lines)


# Angle uncertainty per joint (degrees). Hobby servos: ~5 us deadband
# (0.45 deg) plus metal-gear backlash, ~0.8 deg open loop. With camera
# look-then-correct the backlash is measured away, but a hobby servo still
# moves in deadband-sized steps, so it lands within about half a step.
# Smart serial servos (e.g. Feetech STS3215) have a 4096-count encoder.
LEVELS = [
    ("hobby servos, open loop (this prototype)", 0.8, 0.0),
    ("hobby servos + camera correction", 0.25, 0.15),
    ("smart servos (STS3215), open loop", 0.2, 0.0),
    ("smart servos + camera correction", 0.04, 0.15),
]

# Placement tolerance each kind of job needs (mm)
JOBS = [
    ("modules, motors, M2 screws (with lead-in chamfers)", 1.5),
    ("through-hole LEDs / headers into chamfered holes", 0.5),
    ("iron on a 1.5 mm solder pad", 0.5),
    ("SMD 0805 / 0603 into solder paste, then hot-plate reflow", 0.2),
    ("fine-pitch IC (0.5 mm pitch)", 0.05),
]


def tip_error(cell, arm_name: str = "arm1", at: str = "fixture", tool_len: float = 38.0, err_deg: float = 0.8,
              mode: str = "sum") -> float:
    """Tool-tip error (mm) at a location from err_deg on each of the 4 joints.

    mode "sum" is the worst case (all errors line up), "rss" a typical case.
    """
    from .kinematics import forward

    arm = cell.arms[arm_name]
    l3 = cell.links.coupler + tool_len
    j = inverse(cell, arm, cell.location(at), cell.motion["tool_pitch"], 0, l3)
    p0 = forward(cell, arm, j, l3)
    parts = []
    for i in range(4):
        jj = list(j)
        jj[i] += err_deg
        parts.append(math.dist(forward(cell, arm, jj, l3), p0))
    return sum(parts) if mode == "sum" else math.sqrt(sum(d * d for d in parts))


def precision_levels(cell) -> list[dict]:
    out = []
    for name, err, camera in LEVELS:
        worst = max(max(tip_error(cell, a, err_deg=err) for a in cell.arms), camera)
        typical = max(max(tip_error(cell, a, err_deg=err, mode="rss") for a in cell.arms), camera)
        out.append({"level": name, "worst_mm": round(worst, 2), "typical_mm": round(typical, 2)})
    return out


def precision_report(cell) -> str:
    levels = precision_levels(cell)
    lines = ["Tool-tip accuracy at the fixture (worst of the 4 arms)", "-" * 54,
             f"  {'':<44}{'typical':>9}{'worst':>9}"]
    for lv in levels:
        lines.append(f"  {lv['level']:<44}{lv['typical_mm']:>7.2f}mm{lv['worst_mm']:>7.2f}mm")
    lines += ["", "Cheapest level that can do each job (judged on the typical error)", "-" * 64]
    for job, tol in JOBS:
        ok = [lv["level"] for lv in levels if lv["typical_mm"] <= tol]
        lines.append(f"  {job:<52} +/-{tol:<5.2f} {ok[0] if ok else 'not with arms like these: order the board assembled'}")
    lines += ["", "  Lead-in chamfers, funnels and spring-loaded (compliant) tools let a part find its",
              "  hole even when the arm is off by about the chamfer size."]
    return "\n".join(lines)
