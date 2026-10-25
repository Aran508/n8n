"""Servo envelopes from the datasheets (step.parts was unreachable from the build machine,
so these are dimensioned stand-ins: body, mounting flange with holes, output boss and spline).

Datum: output shaft on +Z through the origin, top face at z = 0, body towards -X."""

from __future__ import annotations

from lib.geo import servo

# name: (L, W, H, shaft offset from near end, flange length, flange t, flange below top,
#        boss r, boss h, spline r, spline h, hole r)
SPECS = {
    "mg996r": (40.7, 19.7, 36.5, 10.2, 54.5, 2.5, 9.5, 6.0, 2.0, 2.9, 4.0, 2.25),
    "mg90s": (22.8, 12.2, 22.5, 5.9, 32.5, 2.5, 4.5, 3.4, 1.5, 2.4, 3.0, 1.0),
    "sg90": (22.5, 11.8, 22.7, 5.9, 32.2, 2.5, 4.6, 3.4, 1.5, 2.4, 3.0, 1.0),
}
TOP_TO_SPLINE_END = {k: v[8] + v[10] for k, v in SPECS.items()}


def servo_body(name: str):
    return servo(*SPECS[name])
