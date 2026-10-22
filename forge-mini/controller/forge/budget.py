"""Reach and static torque check for the arm design (`forge budget`).

Worst case for every pitch joint is the arm stretched out horizontally, so the
torque each servo must hold is sum(mass * horizontal lever arm) of everything
outboard of it. We compare that against the servo's stall torque and want to
stay under 50% (hobby servos overheat and jitter above that).
"""

from __future__ import annotations

from .config import Cell

# Stall torque at 6 V (kg*cm) and mass (g) from typical datasheets.
SERVOS = {
    "MG996R": {"stall": 11.0, "mass": 55.0},
    "MG90S": {"stall": 2.2, "mass": 13.4},
    "SG90": {"stall": 1.8, "mass": 9.0},
}

DESIGN = {"base": "MG996R", "shoulder": "MG996R", "elbow": "MG90S", "wrist": "SG90", "tool": "SG90"}
LINK_MASS_G = {"upper": 8.0, "fore": 7.0, "hand": 6.0}  # printed PLA, 25% infill
PAYLOAD_G = 30.0
SAFE_FRACTION = 0.5


def torque_table(cell: Cell, payload_g: float = PAYLOAD_G) -> list[dict]:
    L = cell.links
    elbow_servo = SERVOS[DESIGN["elbow"]]["mass"]
    wrist_servo = SERVOS[DESIGN["wrist"]]["mass"]
    tool_servo = SERVOS[DESIGN["tool"]]["mass"]
    hand = LINK_MASS_G["hand"] + tool_servo  # gripper + its servo, centred on L3

    # (mass g, distance from the shoulder axis in mm)
    loads = [
        ("upper arm", LINK_MASS_G["upper"], L.L1 / 2),
        ("elbow servo", elbow_servo, L.L1),
        ("forearm", LINK_MASS_G["fore"], L.L1 + L.L2 / 2),
        ("wrist servo", wrist_servo, L.L1 + L.L2),
        ("hand + tool servo", hand, L.L1 + L.L2 + L.L3 / 2),
        ("payload", payload_g, L.L1 + L.L2 + L.L3),
    ]
    pivots = {"shoulder": 0.0, "elbow": L.L1, "wrist": L.L1 + L.L2}

    rows = []
    for joint, pivot in pivots.items():
        g_cm = sum(m * (d - pivot) / 10 for _, m, d in loads if d > pivot)
        need = g_cm / 1000  # kg*cm
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


def reach_summary(cell: Cell) -> dict:
    L = cell.links
    return {
        "stretched_horizontal_mm": L.L1 + L.L2 + L.L3,
        "max_wrist_mm": round(0.95 * (L.L1 + L.L2)),
        "arm_to_plate_centre_mm": 100,
    }


def report(cell: Cell) -> str:
    lines = ["Reach", "-----"]
    for k, v in reach_summary(cell).items():
        lines.append(f"  {k:<26}{v}")
    lines += ["", f"Static torque (arm horizontal, {PAYLOAD_G:.0f} g payload)", "-" * 44]
    lines.append(f"  {'joint':<10}{'servo':<8}{'needs':>8}{'stall':>8}{'load':>7}")
    for r in torque_table(cell):
        flag = "ok" if r["ok"] else "TOO HIGH"
        lines.append(
            f"  {r['joint']:<10}{r['servo']:<8}{r['torque_kgcm']:>6.2f}kg{r['stall_kgcm']:>6.1f}kg"
            f"{r['load_pct']:>6}%  {flag}"
        )
    alt = torque_table_with(cell, "shoulder", "MG90S")
    lines += ["", f"  (with an MG90S on the shoulder it would be {alt}% of stall, so we use an MG996R)"]
    return "\n".join(lines)


def torque_table_with(cell: Cell, joint: str, servo: str) -> int:
    saved = DESIGN[joint]
    DESIGN[joint] = servo
    try:
        return next(r["load_pct"] for r in torque_table(cell) if r["joint"] == joint)
    finally:
        DESIGN[joint] = saved
