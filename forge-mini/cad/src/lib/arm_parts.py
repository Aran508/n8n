"""Printed parts of one arm, modelled in the arm frame at the ready pose (see dims.py).

Layering across Y (the joint axes all run along Y):
  shoulder MG996R  y -44.5 .. -8   (yaw group, shaft +Y into the upper arm)
  upper arm        y  -4   .. 4
  elbow MG996R     y   8   .. 44.5 (forearm group, horn bolted to the upper arm)
  forearm beam     y  20   .. 28
  wrist MG90S      y  -6   .. 16.5 (wrist group, horn bolted to the forearm)
  coupler + tool   centred on y = 0, so the tool point stays in the arm's plane
"""

from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.dims import COUPLER, H0, L1, L2, PLATE_R, PLATE_T
from lib.geo import box, cyl, rbox, slot_xz

ELBOW_Z = H0 + L1            # 135
WRIST_X = L2                 # 80
FACE_Z = ELBOW_Z - COUPLER   # 113, coupler face (tool plate top)
M3 = 1.7
HORN_R, HORN_D = 6.5, 2.5    # MG996R horn pocket
HORN90_R = 4.0               # MG90S horn pocket
BALL_R = 5.5                 # pitch radius of the three 6 mm coupler balls

# pedestal foot bolt pattern (arm frame), shared with the work plate's holes
PEDESTAL_HOLES = ((-38.0, -18.0), (-38.0, 18.0), (18.0, -18.0), (18.0, 18.0))


def pedestal() -> bd.Shape:
    """Housing for the base-yaw MG996R; bolts to the work plate with 4 x M3."""
    shell = rbox(-41, 21, -16, 16, 0, 34, 4) - box(-38, 18, -13, 13, 3, 35) - box(-38, 18, -13, 13, -1, 3.5)
    foot = rbox(-44, 24, -22, 22, 0, 4, 5)
    part = shell + foot - box(-38, 18, -13, 13, -1, 35)
    part = part + box(-38, -33, -13, 13, 0, 24.5) + box(13, 18, -13, 13, 0, 24.5)  # servo flange seats
    for x, y in PEDESTAL_HOLES:
        part = part - cyl(M3, "z", (x, y, -1), 6)
    part = part - box(-44, -38, -5, 5, 4, 14)          # cable exit at the back
    return part


def turntable() -> bd.Shape:
    """Yaw plate on the base servo horn; carries the shoulder bracket."""
    part = rbox(-34, 16, -50, 12, 40, 44, 6) - cyl(HORN_R, "z", (0, 0, 39), HORN_D + 1)
    for x, y in ((-28, -42), (-28, 6), (10, -42), (10, 6)):
        part = part - cyl(M3, "z", (x, y, 39), 6)
    return part


def shoulder_bracket() -> bd.Shape:
    """Two cheeks and a back web: the outer cheek holds the shoulder servo,
    the inner cheek carries the idler pin through the upper arm."""
    outer = box(-34, 14, -48.5, -44.5, 44, 66)
    inner = box(-11, 11, 6, 10, 44, 66) + cyl(11, "y", (0, 6, H0), 4)
    web = box(-34, -31, -48.5, 10, 44, 50) - box(-35, -30, -20.2, -17.3, 43, 51)   # notch for the servo flange
    part = outer + inner + web - cyl(2.0, "y", (0, 5, H0), 6)
    for x in (-25, 5):  # servo flange screws through the outer cheek
        part = part - cyl(M3, "y", (x, -49, H0), 6)
    return part


def upper_arm() -> bd.Shape:
    """L1 = 80 mm link: shoulder horn pocket on the -Y face, elbow horn pocket on the +Y face."""
    part = slot_xz((0, H0), (0, ELBOW_Z), 10, -4, 4)
    part = part - slot_xz((0, H0 + 22), (0, ELBOW_Z - 22), 4, -5, 5)             # lightening slot
    part = part - cyl(HORN_R, "y", (0, -4.1, H0), HORN_D + 0.1)                   # shoulder horn
    part = part - cyl(HORN_R, "y", (0, 4.1, ELBOW_Z), -(HORN_D + 0.1))            # elbow horn
    part = part - cyl(1.6, "y", (0, -5, H0), 10) - cyl(1.6, "y", (0, -5, ELBOW_Z), 10)
    return part


def forearm() -> bd.Shape:
    """L2 = 80 mm link: a strap cradling the elbow servo, an end block and a beam out to the wrist."""
    strap = rbox(-14, 40, 44.5, 48.5, 124, 146, 0)
    end = box(30.5, 40, 8, 48.5, 125, 145) - box(30.4, 37.6, 17.3, 20.2, 124, 146)   # slot for the servo flange tab
    beam = box(30.5, WRIST_X, 20, 28, 128, 142) + cyl(9, "y", (WRIST_X, 20, ELBOW_Z), 8)
    part = strap + end + beam
    part = part - cyl(HORN90_R, "y", (WRIST_X, 19.9, ELBOW_Z), 2.6) - cyl(1.2, "y", (WRIST_X, 19, ELBOW_Z), 10)
    part = part - slot_xz((44, ELBOW_Z), (WRIST_X - 16, ELBOW_Z), 3, 19, 29)      # lightening slot
    return part


def wrist_hub() -> bd.Shape:
    """Hangs under the wrist servo and carries the arm half of the tool changer:
    the coupler disc (face at z = 113) with three ball seats and four pogo pins."""
    stem = rbox(WRIST_X - 7, WRIST_X + 7, -7, 7, FACE_Z + PLATE_T, ELBOW_Z - 6.1, 2)
    disc = cyl(PLATE_R, "z", (WRIST_X, 0, FACE_Z), PLATE_T)
    part = stem + disc - box(72.8, 77.9, -6.2, 6.2, 123.4, 126.3)   # slot for the latch servo's flange tab
    for k in range(3):  # ball seats
        a = math.radians(90 + 120 * k)
        part = part - bd.Pos(WRIST_X + BALL_R * math.cos(a), BALL_R * math.sin(a), FACE_Z + 1) * bd.Sphere(3.05)
    for dx, dy in ((-2.5, -2.5), (2.5, -2.5), (-2.5, 2.5), (2.5, 2.5)):  # pogo pin bores
        part = part - cyl(0.8, "z", (WRIST_X + dx, dy, FACE_Z - 0.1), 6)
    return part


def coupler_balls() -> bd.Shape:
    balls = [bd.Pos(WRIST_X + BALL_R * math.cos(math.radians(90 + 120 * k)),
                    BALL_R * math.sin(math.radians(90 + 120 * k)), FACE_Z + 1) * bd.Sphere(3.0) for k in range(3)]
    return bd.Compound(children=balls)


def latch_hook() -> bd.Shape:
    """Cam hook on the latch servo: swings under the tool plate's rim to lock it (105 deg)."""
    return box(65, 69, -2.5, 2.5, 107.5, 114.5) + box(65, 73.5, -2.5, 2.5, 107.5, 109)
