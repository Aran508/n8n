"""Cell furniture: tool racks, bit block, flip fixture, work plate, kit drawer, trays, enclosure.
All in world coordinates unless a docstring says otherwise."""

from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.arm_parts import PEDESTAL_HOLES
from lib.dims import (ACRYLIC_T, ARMS, BIT_BLOCK_X, BIT_TOP_Z, BITS, CUPS, ENCL_H, ENCL_SIZE, EXTRUSION,
                      FINISHED, FIXTURE_TOP_Z, FLIP_AXIS_Z, FLOOR_Z, FOOT_H, KIT_TRAY, PLATE_R, PLATE_SIZE,
                      PLATE_T_BASE, RACK_SLOTS_Y, RACK_Z, SIDE_PART, PLATE_T, WINDOW_H, WINDOW_W)
from lib.geo import box, cone, cyl, rbox

SHELF_TOP = RACK_Z - PLATE_T   # the tool plate rests on the shelf, its face at RACK_Z
SLOT_HOLE_R = 7.5              # tool bodies fit a 15 mm circle, the 17.4 mm plate does not

# work plate openings (world)
FIXTURE_WINDOW = (-54.0, 54.0, -27.0, 27.0)    # through, so the cradle can swing below the plate
FIXTURE_REBATE = (-58.0, 58.0, -31.0, 31.0)    # the fixture's base frame sits in this from below
REBATE_DEPTH = 3.0
RACK_WELL = (10.0, 46.0)                       # half-width x, half-length y of the slot under each rack


# ---------------------------------------------------------------- racks
def rack() -> bd.Shape:
    """Five-slot tool rack in its own frame: slots along Y at x = 0, posts on the +X (outer) side."""
    shelf = rbox(-12, 16, -56, 56, SHELF_TOP - 4, SHELF_TOP, 4)
    for y in RACK_SLOTS_Y:
        shelf = shelf - cyl(SLOT_HOLE_R, "z", (0, y, SHELF_TOP - 5), 6)
    rail = box(12, 16, -56, 56, 0, SHELF_TOP - 4)
    posts = box(4, 16, 48, 56, 0, SHELF_TOP - 4) + box(4, 16, -56, -48, 0, SHELF_TOP - 4)
    return shelf + rail + posts


def rack_b() -> bd.Shape:
    """One-slot stand for the second iron, in its own frame (slot at the origin)."""
    shelf = rbox(-18, 18, -10, 10, SHELF_TOP - 4, SHELF_TOP, 3)
    shelf = shelf - cyl(SLOT_HOLE_R, "z", (0, 0, SHELF_TOP - 5), 6)
    posts = box(-18, -12, -6, 6, 0, SHELF_TOP - 4) + box(12, 18, -6, 6, 0, SHELF_TOP - 4)
    return shelf + posts


# ---------------------------------------------------------------- bit block
BLOCK_TOP = 14.0   # bit collars rest here; the shank stands up to BIT_TOP_Z for the collet


def bit_block() -> bd.Shape:
    part = rbox(BIT_BLOCK_X - 8, BIT_BLOCK_X + 8, -32, 32, 0, BLOCK_TOP, 3)
    for _, dia, _, y in BITS.values():
        part = part - cyl(dia / 2 + 0.3, "z", (BIT_BLOCK_X, y, -1), BLOCK_TOP + 2)
        part = part - cyl(3.4, "z", (BIT_BLOCK_X, y, BLOCK_TOP - 1), 1.1)                # collar seat
    return part


def bit(kind: str, dia: float, stickout: float, y: float) -> bd.Shape:
    """A bit standing shank-up in the block: cutting end down, collar, 6 mm shank for the collet."""
    x = BIT_BLOCK_X
    collar = cyl(3.2, "z", (x, y, BLOCK_TOP - 1), 1.0)
    shank = cyl(1.5, "z", (x, y, BLOCK_TOP), BIT_TOP_Z - BLOCK_TOP)
    bottom = BLOCK_TOP - 1 - (stickout - (BIT_TOP_Z - BLOCK_TOP))
    if kind == "countersink":
        cut = cone(0.6, dia / 2, (x, y, bottom), BLOCK_TOP - 1 - bottom)
    else:
        cut = cyl(dia / 2, "z", (x, y, bottom), BLOCK_TOP - 1 - bottom)
    return collar + shank + cut


# ---------------------------------------------------------------- flip fixture
CRADLE_X = 26.0
POST_X = (38.0, 48.0)
BEARING = (11.0, 4.0, 7.0)    # 608: OD/2, bore/2, width
SHAFT_R = 4.0


def fixture_base() -> bd.Shape:
    x0, x1, y0, y1 = FIXTURE_REBATE
    z0 = -PLATE_T_BASE
    frame = box(x0, x1, y0, y1, z0, z0 + REBATE_DEPTH) - box(-32, 32, -27, 27, z0 - 1, z0 + REBATE_DEPTH + 1)
    for sx in (-1, 1):
        post = box(sx * POST_X[0], sx * POST_X[1], -14, 14, z0 + REBATE_DEPTH, FLIP_AXIS_Z + 12)
        post = post - cyl(BEARING[0], "x", (sx * POST_X[0], 0, FLIP_AXIS_Z), sx * BEARING[2])
        post = post - cyl(SHAFT_R + 0.3, "x", (sx * (POST_X[0] - 1), 0, FLIP_AXIS_Z), sx * 12)
        frame = frame + post
    return frame


def bearings() -> bd.Shape:
    rings = []
    for sx in (-1, 1):
        ring = cyl(BEARING[0], "x", (sx * POST_X[0], 0, FLIP_AXIS_Z), sx * BEARING[2])
        rings.append(ring - cyl(BEARING[1], "x", (sx * (POST_X[0] - 1), 0, FLIP_AXIS_Z), sx * (BEARING[2] + 2)))
    return rings[0] + rings[1]


def cradle() -> bd.Shape:
    """Turns about X at FLIP_AXIS_Z. Holds a 40 x 40 board on a ledge so its top is at FIXTURE_TOP_Z;
    the 36 x 36 opening leaves every leg reachable from below once flipped."""
    board_bottom = FIXTURE_TOP_Z - 1.6
    ring = box(-CRADLE_X, CRADLE_X, -23, 23, -1, 4) - box(-18, 18, -18, 18, -2, 5) - box(-20.3, 20.3, -20.3, 20.3, board_bottom, 5)
    cheeks = box(-30, -CRADLE_X, -7, 7, -1, 14) + box(CRADLE_X, 30, -7, 7, -1, 14)
    shafts = cyl(SHAFT_R, "x", (30, 0, FLIP_AXIS_Z), POST_X[1] - 30) + cyl(SHAFT_R, "x", (-30, 0, FLIP_AXIS_Z), -(44 - 30))
    return ring + cheeks + shafts


def watch_board() -> list[tuple[bd.Shape, str, str]]:
    """The smartwatch perfboard with its parts placed (make_smartwatch.py layout)."""
    z = FIXTURE_TOP_Z
    pcb = box(-20, 20, -20, 20, z - 1.6, z)
    for i in range(15):
        for j in range(15):
            pcb = pcb - cyl(0.5, "z", (-17.78 + i * 2.54, -17.78 + j * 2.54, z - 2), 3)
    esp = box(-3 - 11.25, -3 + 11.25, 9 - 9, 9 + 9, z + 2.5, z + 5.5)
    chg = box(14 - 5.5, 14 + 5.5, 9 - 5.5, 9 + 5.5, z, z + 1.6) + box(14 - 4.5, 14 + 4.5, 12.5, 15.5, z + 1.6, z + 4.8)
    buttons = [box(bx - 6, bx + 6, -20, -8, z, z + 3.5) + cyl(1.8, "z", (bx, -14, z + 3.5), 1.5) for bx in (-12, 0, 12)]
    sock = box(-6, 6, -4.3, -1.7, z, z + 8.5)
    return [(pcb, "watch_perfboard", "#2e7d4f"), (esp, "watch_esp32c3", "#1d1f22"), (chg, "watch_charger", "#2f6fd6"),
            (buttons[0] + buttons[1] + buttons[2], "watch_buttons", "#d9d9d9"), (sock, "watch_oled_socket", "#1d1f22")]


# ---------------------------------------------------------------- work plate
def _arm_world(name: str, x: float, y: float) -> tuple[float, float]:
    bx, by, yaw, _ = ARMS[name]
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return bx + x * c - y * s, by + x * s + y * c


def work_plate() -> bd.Shape:
    h = PLATE_SIZE / 2
    plate = rbox(-h, h, -h, h, -PLATE_T_BASE, 0, 8)
    x0, x1, y0, y1 = FIXTURE_WINDOW
    plate = plate - box(x0, x1, y0, y1, -PLATE_T_BASE - 1, 1)
    x0, x1, y0, y1 = FIXTURE_REBATE
    plate = plate - box(x0, x1, y0, y1, -PLATE_T_BASE - 1, -PLATE_T_BASE + REBATE_DEPTH)
    hx, hy = RACK_WELL
    for sx in (-1, 1):
        plate = plate - rbox(sx * 100 - hx, sx * 100 + hx, -hy, hy, -PLATE_T_BASE - 1, 1, 4)
    plate = plate - cyl(SLOT_HOLE_R + 2, "z", (0, 90, -PLATE_T_BASE - 1), PLATE_T_BASE + 2)
    for _, dia, stickout, y in BITS.values():
        plate = plate - cyl(2.5, "z", (BIT_BLOCK_X, y, -PLATE_T_BASE - 1), PLATE_T_BASE + 2)
    for name in ARMS:
        for x, y in PEDESTAL_HOLES:
            plate = plate - cyl(1.7, "z", (*_arm_world(name, x, y), -PLATE_T_BASE - 1), PLATE_T_BASE + 2)
    for gx in range(-140, 141, 20):   # engraved 20 mm grid, the camera's calibration reference
        if abs(gx) < 130:
            plate = plate - box(gx - 0.3, gx + 0.3, -140, 140, -0.4, 0.1) - box(-140, 140, gx - 0.3, gx + 0.3, -0.4, 0.1)
    return plate


def feet() -> bd.Shape:
    f = [cyl(10, "z", (sx * 130, sy * 130, -PLATE_T_BASE - FOOT_H), FOOT_H) for sx in (-1, 1) for sy in (-1, 1)]
    return f[0] + f[1] + f[2] + f[3]


# ---------------------------------------------------------------- loading: kit drawer, cups, trays
DRAWER_Z = 2.0
DRAWER = (-45.0, 45.0, -206.0, -78.0)


def drawer_rails() -> bd.Shape:
    """Two runners on the plate, carried past its front edge on legs to the bench."""
    out = None
    for sx in (-1, 1):
        r = box(sx * 46, sx * 50, -200, -86, 0, DRAWER_Z) + box(sx * 46, sx * 50, -184, -152, FLOOR_Z, 0)
        out = r if out is None else out + r
    return out


def kit_drawer() -> bd.Shape:
    x0, x1, y0, y1 = DRAWER
    tray = rbox(x0, x1, y0, y1, DRAWER_Z, DRAWER_Z + 8, 4)
    kx, ky = KIT_TRAY
    tray = tray - box(kx - 22, kx + 22, ky - 22, ky + 22, DRAWER_Z + 2, DRAWER_Z + 9)       # kit pocket (plate/board)
    sx, sy = SIDE_PART
    tray = tray - box(sx - 9, sx + 9, sy - 10, sy + 10, DRAWER_Z + 2, DRAWER_Z + 9)          # module pocket
    cx, cy = CUPS["cup_c"]
    tray = tray - cyl(9.3, "z", (cx, cy, DRAWER_Z + 2), 8)                                    # cup_c seat
    handle = box(-25, 25, y0 - 8, y0, DRAWER_Z + 2, DRAWER_Z + 6) + box(-25, -21, y0 - 8, y0 + 2, DRAWER_Z, DRAWER_Z + 6) \
        + box(21, 25, y0 - 8, y0 + 2, DRAWER_Z, DRAWER_Z + 6)
    tag = box(-14, 14, y0 + 2, y0 + 14, DRAWER_Z + 8, DRAWER_Z + 8.4)
    return tray + handle + tag


def cup(x: float, y: float, z0: float) -> bd.Shape:
    return cyl(9, "z", (x, y, z0), 12) - cyl(7.8, "z", (x, y, z0 + 1.2), 12)


def finished_tray() -> bd.Shape:
    x, y = FINISHED
    return rbox(x - 18, x + 18, y - 11, y + 11, 0, 6, 3) - rbox(x - 16, x + 16, y - 9, y + 9, 1.5, 7, 2)


# ---------------------------------------------------------------- enclosure
E = ENCL_SIZE / 2
TOP = FLOOR_Z + ENCL_H
RAIL_Z = FLOOR_Z + EXTRUSION + WINDOW_H + 30    # front rail between the loading strip and the door


def extrusion(axis: str, start: tuple[float, float, float], length: float) -> bd.Shape:
    """2020 profile with its four T-slots, centred on `start` across, running `length` along +axis."""
    h = EXTRUSION / 2
    lo, c = bd.Align.MIN, bd.Align.CENTER
    prof = bd.Box(EXTRUSION, EXTRUSION, length, align=(c, c, lo))
    for a in range(4):
        prof = prof - bd.Rot(0, 0, 90 * a) * bd.Pos(h - 2.5, 0, -1) * bd.Box(5.2, 6.2, length + 2, align=(c, c, lo))
    prof = prof - bd.Pos(0, 0, -1) * bd.Cylinder(2.1, length + 2, align=(c, c, lo))
    rot = {"x": bd.Rot(0, 90, 0), "y": bd.Rot(-90, 0, 0), "z": bd.Rot(0, 0, 0)}[axis]
    return bd.Pos(*start) * rot * prof


def frame() -> bd.Shape:
    h, e = EXTRUSION / 2, E - EXTRUSION / 2
    parts = [extrusion("z", (sx * e, sy * e, FLOOR_Z), ENCL_H) for sx in (-1, 1) for sy in (-1, 1)]
    for z in (FLOOR_Z + h, TOP - h):
        parts += [extrusion("x", (-e + h, sy * e, z), 2 * e - EXTRUSION) for sy in (-1, 1)]
        parts += [extrusion("y", (sx * e, -e + h, z), 2 * e - EXTRUSION) for sx in (-1, 1)]
    parts.append(extrusion("x", (-e + h, -e, RAIL_Z), 2 * e - EXTRUSION))       # front rail above the loading strip
    parts.append(extrusion("x", (-e + h, 0, TOP - h), 2 * e - EXTRUSION))       # roof rail for the camera and light
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def panels() -> bd.Shape:
    """Clear acrylic in the frame's slots: back, sides, roof, and the fixed strip under the door
    with the loading window."""
    e, t, i = E - EXTRUSION / 2, ACRYLIC_T / 2, E - EXTRUSION
    z0, z1 = FLOOR_Z + EXTRUSION, TOP - EXTRUSION
    back = box(-i, i, e - t, e + t, z0, z1)
    left = box(-e - t, -e + t, -i, i, z0, z1)
    right = box(e - t, e + t, -i, i, z0, z1)
    roof = box(-i, i, -i, i, TOP - EXTRUSION / 2 - t, TOP - EXTRUSION / 2 + t)
    strip = box(-i, i, -e - t, -e + t, z0, RAIL_Z - EXTRUSION / 2)
    strip = strip - box(-WINDOW_W / 2, WINDOW_W / 2, -e - 2, -e + 2, z0 - 1, z0 + WINDOW_H)
    fan_hole = cyl(38, "y", (-120, e - 3, 230), 6)
    return back - fan_hole + left + right + roof + strip


DOOR_HINGE = (-E, -E - 4.0)    # x, y of the hinge axis (outside the left front post)


def door() -> bd.Shape:
    x0, y = -E + 4, -E - 4
    z0, z1 = RAIL_Z + EXTRUSION / 2 + 2, TOP - 4
    pane = box(x0, E - 4, y - ACRYLIC_T / 2, y + ACRYLIC_T / 2, z0, z1)
    return pane


def door_hardware() -> bd.Shape:
    """Hinges, the handle and the reed-switch magnet (all on the door)."""
    y = -E - 4
    z0, z1 = RAIL_Z + EXTRUSION / 2 + 2, TOP - 4
    hinges = [cyl(4, "z", (DOOR_HINGE[0], DOOR_HINGE[1], z), 30) for z in (z0 + 20, z1 - 50)]
    handle = box(E - 40, E - 32, y - 22, y - ACRYLIC_T / 2, (z0 + z1) / 2 - 40, (z0 + z1) / 2 + 40) \
        + box(E - 40, E - 32, y - 8, y - ACRYLIC_T / 2, (z0 + z1) / 2 - 50, (z0 + z1) / 2 - 40) \
        + box(E - 40, E - 32, y - 8, y - ACRYLIC_T / 2, (z0 + z1) / 2 + 40, (z0 + z1) / 2 + 50)
    magnet = box(E - 34, E - 24, y + ACRYLIC_T / 2, -E + 4, (z0 + z1) / 2 - 8, (z0 + z1) / 2 + 8)
    return hinges[0] + hinges[1] + handle + magnet


def reed_switch() -> bd.Shape:
    """Door interlock: the reed switch on the right front post, facing the door's magnet."""
    z = (RAIL_Z + EXTRUSION / 2 + 2 + TOP - 4) / 2
    return box(E - 24, E - 20, -E, -E + 12, z - 10, z + 10)


def light_curtain() -> list[bd.Shape]:
    """Emitter and receiver bars either side of the loading window, six beams 10 mm apart."""
    z0 = FLOOR_Z + EXTRUSION
    y = -E - 4
    bars = []
    for sx in (-1, 1):
        bar = box(sx * (WINDOW_W / 2 + 2), sx * (WINDOW_W / 2 + 10), y - 6, y + 2, z0, z0 + WINDOW_H)
        for k in range(6):
            bar = bar - cyl(1.5, "x", (sx * (WINDOW_W / 2 + 1), y - 2, z0 + 5 + 10 * k), sx * 2)
        bars.append(bar)
    return bars


def tower_light() -> list[tuple[bd.Shape, str, str]]:
    x, y = E - EXTRUSION / 2, E - EXTRUSION / 2
    pole = cyl(4, "z", (x, y, TOP), 30)
    base = cyl(14, "z", (x, y, TOP), 6)
    out = [(pole + base, "tower_pole", "#2b2f36")]
    for k, (name, colour) in enumerate((("green", "#2fbf5b"), ("yellow", "#f2c31e"), ("red", "#d92b25"))):
        out.append((cyl(11, "z", (x, y, TOP + 30 + 20 * k), 18), f"tower_{name}", colour))
    out.append((cone(11, 4, (x, y, TOP + 90), 6), "tower_cap", "#2b2f36"))
    return out


def roof_gear() -> list[tuple[bd.Shape, str, str]]:
    z = TOP - EXTRUSION
    led = box(-150, 150, -7 + 30, 7 + 30, z - 4, z)
    cam = box(-15, 15, -15, 15, z - 22, z) + cyl(6, "z", (0, 0, z - 28), 6)
    return [(led, "overhead_led_bar", "#f5f1e6"), (cam, "camera_1080p", "#1d1f22")]


def fan() -> list[tuple[bd.Shape, str, str]]:
    e = E - EXTRUSION / 2
    housing = box(-160, -80, e + 2, e + 27, 190, 270) - cyl(37, "y", (-120, e + 1, 230), 30)
    blades = cyl(12, "y", (-120, e + 8, 230), 10)
    for k in range(7):
        blades = blades + bd.Pos(-120, e + 13, 230) * bd.Rot(0, 360 / 7 * k, 0) * bd.Rot(0, 0, 25) * box(-3, 3, -2, 2, 12, 35)
    return [(housing, "fume_fan", "#1d1f22"), (blades, "fan_blades", "#2b2f36")]


def console() -> list[tuple[bd.Shape, str, str]]:
    """Operator console to the right of the cell: 7 inch touchscreen, START, STOP, E-STOP."""
    x0, x1, y0, y1 = E + 30, E + 150, -E, -E + 110
    z0 = FLOOR_Z
    body = box(x0, x1, y0, y1, z0, z0 + 70)
    slope = bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, z0 + 70) * bd.Rot(28, 0, 0) * bd.Box(x1 - x0, 112, 6)
    screen = bd.Pos((x0 + x1) / 2, (y0 + y1) / 2 + 12, z0 + 70) * bd.Rot(28, 0, 0) * bd.Pos(0, 0, 3.5) * bd.Box(100, 60, 1.5)
    btn_y, btn_z = y0, z0 + 40
    start = cyl(7, "y", (x0 + 30, btn_y - 1, btn_z), -6)
    stop = cyl(7, "y", (x0 + 60, btn_y - 1, btn_z), -6)
    ring = cyl(14, "y", (x0 + 95, btn_y - 0.1, btn_z), -3)
    estop = cyl(10, "y", (x0 + 95, btn_y - 3, btn_z), -6) + cyl(12, "y", (x0 + 95, btn_y - 9, btn_z), -6)
    return [(body + slope, "console", "#3a3f47"), (screen, "hmi_7in", "#0b1e2e"), (start, "start_button", "#2fbf5b"),
            (stop, "stop_button", "#1d1f22"), (ring, "estop_ring", "#f2c31e"), (estop, "estop", "#d92b25")]


def floor_marking() -> bd.Shape:
    o, i, z = E + 24, E + 8, FLOOR_Z
    return box(-o, o, -o, o, z - 1, z) - box(-i, i, -i, i, z - 2, z + 1)
