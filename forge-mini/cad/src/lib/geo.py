"""Small geometry helpers shared by every Forge Mini model."""

from __future__ import annotations

from cadgen import build123d as bd
from cadgen import srgb

import math

PRINT_GREY = "#3a3f47"
SERVO_BLACK = "#1d1f22"
SG90_BLUE = "#2f6fd6"
STEEL = "#b8bec6"
BRASS = "#c9a24a"
PLATE_DARK = "#2b2f36"
ALU = "#a9afb7"


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> bd.Solid:
    """Axis-aligned box from its corner coordinates."""
    lo = bd.Align.MIN
    return bd.Pos(min(x0, x1), min(y0, y1), min(z0, z1)) * bd.Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), align=(lo, lo, lo))


def rbox(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float, r: float) -> bd.Solid:
    """Box with its vertical edges rounded."""
    b = box(x0, x1, y0, y1, z0, z1)
    return bd.fillet(b.edges().filter_by(bd.Axis.Z), r) if r > 0 else b


def cyl(r: float, axis: str, start: tuple[float, float, float], length: float) -> bd.Solid:
    """Cylinder of radius r from `start` along +axis ('x', 'y' or 'z'); negative length goes the other way."""
    lo, c = bd.Align.MIN, bd.Align.CENTER
    c0 = bd.Cylinder(r, abs(length), align=(c, c, lo))
    if length < 0:
        c0 = bd.Pos(0, 0, length) * c0
    rot = {"x": bd.Rot(0, 90, 0), "y": bd.Rot(-90, 0, 0), "z": bd.Rot(0, 0, 0)}[axis]
    return bd.Pos(*start) * rot * c0


def cone(r0: float, r1: float, start: tuple[float, float, float], length: float) -> bd.Solid:
    """Vertical cone from radius r0 at `start` to r1, `length` along +Z (negative = downwards)."""
    lo, c = bd.Align.MIN, bd.Align.CENTER
    if length >= 0:
        return bd.Pos(*start) * bd.Cone(r0, r1, length, align=(c, c, lo))
    return bd.Pos(start[0], start[1], start[2] + length) * bd.Cone(r1, r0, -length, align=(c, c, lo))


def slot_xz(p0: tuple[float, float], p1: tuple[float, float], r: float, y0: float, y1: float) -> bd.Solid:
    """A flat bar in the XZ plane with round ends at p0 and p1 (x, z), from y0 to y1."""
    (x0, z0), (x1, z1) = p0, p1
    length = math.hypot(x1 - x0, z1 - z0)
    ang = math.degrees(math.atan2(z1 - z0, x1 - x0))
    bar = box(0, length, y0, y1, -r, r) + cyl(r, "y", (0, y0, 0), y1 - y0) + cyl(r, "y", (length, y0, 0), y1 - y0)
    return bd.Pos(x0, 0, z0) * bd.Rot(0, -ang, 0) * bar


def paint(shape, colour: str, label: str | None = None, alpha: float | None = None):
    shape.color = srgb(colour) if alpha is None else srgb(colour, alpha)
    if label:
        shape.label = label
    return shape


def group(label: str, children: list) -> bd.Compound:
    return bd.Compound(children=children, label=label)


def servo(L: float, W: float, H: float, off: float, flange_len: float, flange_t: float, flange_from_top: float,
          boss_r: float, boss_h: float, spline_r: float, spline_h: float, hole_r: float) -> bd.Solid:
    """Hobby servo envelope. Output shaft on +Z through the origin; the top face is z = 0,
    the body runs towards -X from the shaft (shaft `off` from the near end) and down to z = -H."""
    body = box(-(L - off), off, -W / 2, W / 2, -H, 0)
    cx = (off - (L - off)) / 2
    flange = box(cx - flange_len / 2, cx + flange_len / 2, -W / 2, W / 2, -flange_from_top - flange_t, -flange_from_top)
    s = body + flange
    for h in [cyl(hole_r, "z", (cx + sx * (flange_len / 2 - (flange_len - L) / 4), sy * W / 4, -flange_from_top - flange_t - 1), flange_t + 2)
              for sx in (-1, 1) for sy in (-1, 1)]:
        s = s - h
    return s + cyl(boss_r, "z", (0, 0, 0), boss_h) + cyl(spline_r, "z", (0, 0, boss_h), spline_h)
