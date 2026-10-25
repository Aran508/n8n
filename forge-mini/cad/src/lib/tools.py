"""End effectors. Datum: the coupler face (top of the tool plate) at the origin, the tool
hanging down -Z, with its tool centre point at z = -length (cell.yaml tools.*.length).

Every tool has the same plate: 17.4 mm across, 4 mm thick, three seats for the arm's steel
balls, four pogo pads, and the rim the latch hook catches. Bodies are kept inside a
15 mm circle (11 x 8 for boxes) so they drop through the 15 mm rack holes."""

from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.dims import PLATE_R, PLATE_T, TOOL_LENGTH
from lib.geo import BRASS, PRINT_GREY, SERVO_BLACK, STEEL, box, cone, cyl, paint


def tool_plate() -> bd.Shape:
    plate = cyl(PLATE_R, "z", (0, 0, -PLATE_T), PLATE_T)
    for k in range(3):  # radial V-groove seats for the 6 mm balls
        a = 90 + 120 * k
        groove = bd.Rot(0, 0, a) * (bd.Pos(5.5, 0, 0) * bd.Rot(45, 0, 0) * bd.Box(6, 2.6, 2.6))
        plate = plate - groove
    for dx, dy in ((-2.5, -2.5), (2.5, -2.5), (-2.5, 2.5), (2.5, 2.5)):
        plate = plate - cyl(0.9, "z", (dx, dy, -0.4), 0.5)
    return plate


def _body(kind: str) -> list[tuple[bd.Shape, str, str]]:
    """(shape, label, colour) pieces below the plate, top at z = -PLATE_T."""
    L = TOOL_LENGTH[kind]
    t = -PLATE_T
    if kind == "gripper":
        housing = box(-5.5, 5.5, -4, 4, t, -20)
        jaws = [box(sx * 2.0, sx * 5.0, -3.5, 3.5, -20, -L) for sx in (-1, 1)]
        return [(housing, "housing", PRINT_GREY), (jaws[0] + jaws[1], "jaws", SERVO_BLACK)]
    if kind == "tweezers":
        grip = cyl(6, "z", (0, 0, t), -14)
        prongs = []
        for sx in (-1, 1):
            p = bd.Pos(sx * 2.2, 0, -18) * bd.Rot(0, sx * 4, 0) * bd.Box(1.2, 3, 2 * (L - 18) - 4, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MAX))
            prongs.append(p & box(-8, 8, -8, 8, -18, -L))
        return [(grip, "grip", PRINT_GREY), (prongs[0] + prongs[1], "prongs", STEEL)]
    if kind == "vacuum":
        body = cyl(5, "z", (0, 0, t), -20)
        nozzle = cyl(1.5, "z", (0, 0, -24), -6)
        cup = cone(2.0, 4.0, (0, 0, -30), -4)
        return [(body, "pump_body", PRINT_GREY), (nozzle, "nozzle", STEEL), (cup, "suction_cup", SERVO_BLACK)]
    if kind == "pen":
        body = cyl(5, "z", (0, 0, t), -32)
        tip = cone(4.2, 0.4, (0, 0, -36), -10)
        return [(body, "holder", PRINT_GREY), (tip, "pen_tip", SERVO_BLACK)]
    if kind == "probe":
        body = cyl(5, "z", (0, 0, t), -20)
        needle = cyl(0.8, "z", (0, 0, -24), -17.5)
        ball = bd.Pos(0, 0, -L + 1.5) * bd.Sphere(1.5)
        return [(body, "switch_body", PRINT_GREY), (needle + ball, "stylus", STEEL)]
    if kind == "clamp":
        body = box(-5.5, 5.5, -4, 4, t, -22)
        pad = box(-5.5, 5.5, -4, 4, -22, -L)
        return [(body, "arm", PRINT_GREY), (pad, "rubber_pad", SERVO_BLACK)]
    if kind == "spindle":
        motor = cyl(6, "z", (0, 0, t), -24)
        collet = cone(4.0, 2.5, (0, 0, -28), -8)
        return [(motor, "n20_motor", STEEL), (collet, "magnetic_collet", BRASS)]
    if kind == "iron":
        handle = cyl(6, "z", (0, 0, t), -24)
        sleeve = cyl(3, "z", (0, 0, -28), -14)
        tip = cone(1.6, 0.35, (0, 0, -42), -10)
        feeder = cyl(0.7, "z", (6.75, 0, t), -42)
        return [(handle, "heater_handle", PRINT_GREY), (sleeve + tip, "tip", STEEL), (feeder, "solder_feed_tube", BRASS)]
    if kind == "dispenser":
        barrel = cyl(5.5, "z", (0, 0, t), -32)
        hub = cone(3.0, 1.2, (0, 0, -36), -3)
        needle = cyl(0.45, "z", (0, 0, -39), -(L - 39))
        return [(barrel, "syringe", PRINT_GREY), (hub, "luer_hub", BRASS), (needle, "needle", STEEL)]
    raise ValueError(kind)


def tool_shape(kind: str) -> bd.Compound:
    parts = [paint(tool_plate(), PRINT_GREY, "tool_plate")]
    for shape, label, colour in _body(kind):
        parts.append(paint(shape, colour, label))
    return bd.Compound(children=parts, label=f"tool_{kind}")
