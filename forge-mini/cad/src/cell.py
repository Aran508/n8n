"""Forge Mini: the whole miniature cell, laid out from controller/cell.yaml.

Built with every arm at its park pose, set up for the smartwatch job: arm1 holds the
tweezers, arm2 the second iron, arm3 the vacuum pen and arm4 the iron; the other seven
tools wait in the racks.

Sliders: armN_base / _shoulder / _elbow / _wrist (joint angle minus the park pose),
flip (0-180 deg), door (0-110 deg), drawer (mm out of the cell).
Poses: park, ready, solder_underside, pick_parts, loading."""

from __future__ import annotations

import cadgen
from cadgen import build123d as bd
from cadgen import step

from bit_block import bit_block
from enclosure import DOOR_MATE, enclosure
from flip_fixture import flip_fixture
from kit_drawer import kit_drawer
from lib import cell_parts as cp
from lib.arm import arm_mates, build_arm, pose
from lib.dims import ARMS, BITS, CUPS, FLIP_AXIS_Z, RACK_Z, READY, TOOLS
from lib.geo import STEEL, group, paint
from rack import rack
from rack_b import rack_b
from tool_clamp import tool_clamp
from tool_dispenser import tool_dispenser
from tool_gripper import tool_gripper
from tool_iron import tool_iron
from tool_pen import tool_pen
from tool_probe import tool_probe
from tool_spindle import tool_spindle
from tool_tweezers import tool_tweezers
from tool_vacuum import tool_vacuum
from work_plate import work_plate

TOOL_MODELS = {"gripper": tool_gripper, "tweezers": tool_tweezers, "vacuum": tool_vacuum, "pen": tool_pen,
               "probe": tool_probe, "clamp": tool_clamp, "spindle": tool_spindle, "iron": tool_iron,
               "dispenser": tool_dispenser}
MOUNTED = {"arm1": "tweezers", "arm2": "iron_B", "arm3": "vacuum", "arm4": "iron"}

PARK = {"base": 0.0, "shoulder": 103.2, "elbow": -142.7, "wrist": -50.5}   # cell.yaml park, from forge ik


def _all(joints_by_arm: dict[str, dict[str, float]]) -> dict[str, float]:
    out: dict[str, float] = {}
    for name in ARMS:
        out |= pose(name, joints_by_arm.get(name, PARK), built=PARK)
    return out


KINEMATICS = {
    "mates": [m for name, (bx, by, yaw, _) in ARMS.items() for m in arm_mates(name, bx, by, yaw, PARK)] + [
        cadgen.revolute("flip", parent="#flip_base", child="#flip_cradle",
                        origin=(0, 0, FLIP_AXIS_Z), direction=(1, 0, 0), limits=(0, 180)),
        DOOR_MATE,
        cadgen.slider("drawer", parent="#work_plate", child="#kit_drawer_group",
                      origin=(0, -100, 0), direction=(0, -1, 0), limits=(0, 120)),
    ],
    "poses": {
        "park": {},
        "ready": _all({name: READY for name in ARMS}),
        # forge ik: irons on the flipped watch board (watch.u.esp1 and watch.u.b11)
        "solder_underside": {**_all({"arm4": {"base": 5.0, "shoulder": 63.9, "elbow": -88.7, "wrist": -65.2},
                                     "arm2": {"base": -12.9, "shoulder": 67.6, "elbow": -94.1, "wrist": -63.5}}),
                             "flip": 180},
        # forge ik: vacuum over side.part, tweezers over cup_b
        "pick_parts": _all({"arm3": {"base": -81.6, "shoulder": 99.7, "elbow": -142.0, "wrist": -47.7},
                            "arm1": {"base": 78.4, "shoulder": 85.5, "elbow": -116.2, "wrist": -59.4}}),
        "loading": {**_all({}), "door": 100, "drawer": 120},
    },
}


def _tool(spec_name: str):
    kind = TOOLS[spec_name][0]
    return TOOL_MODELS[kind]()


@step(out="../STEP/cell.step", kinematics=KINEMATICS)
def cell():
    plate = work_plate()
    plate.label = "work_plate"
    parts = [plate, paint(cp.feet(), "#1d1f22", "feet")]

    for name, (bx, by, yaw, colour) in ARMS.items():
        tool_name = MOUNTED[name]
        arm = build_arm(name, colour, _tool(tool_name), f"{name}_tool_{tool_name}", joints=PARK)
        parts.append(bd.Pos(bx, by, 0) * bd.Rot(0, 0, yaw) * arm)

    r = rack()
    for label, loc in (("rack_R", bd.Pos(100, 0, 0)), ("rack_L", bd.Pos(-100, 0, 0) * bd.Rot(0, 0, 180))):
        placed = loc * r
        placed.label = label
        parts.append(placed)
    rb = bd.Pos(*TOOLS["iron_B"][2:4], 0) * rack_b()
    rb.label = "rack_B"
    parts.append(rb)
    mounted = set(MOUNTED.values())
    for tool_name, (kind, _rack, x, y, _length) in TOOLS.items():
        if tool_name in mounted:
            continue
        t = bd.Pos(x, y, RACK_Z) * _tool(tool_name)
        t.label = f"rack_tool_{tool_name}"
        parts.append(t)

    block = bit_block()
    block.label = "bit_block"
    parts += [block, group("bits", [paint(cp.bit(kind, dia, stick, y), STEEL, f"bit_{name}")
                                    for name, (kind, dia, stick, y) in BITS.items()])]

    fixture = flip_fixture()
    fixture.label = "flip_fixture"
    parts.append(fixture)

    drawer = kit_drawer()
    drawer.label = "kit_drawer"
    cx, cy = CUPS["cup_c"]
    parts += [paint(cp.drawer_rails(), "#a9afb7", "drawer_rails"),
              group("kit_drawer_group", [drawer, paint(cp.cup(cx, cy, cp.DRAWER_Z + 2), "#e8e8e8", "cup_c")])]
    parts += [paint(cp.cup(*CUPS[c], 0), "#e8e8e8", c) for c in ("cup_a", "cup_b")]
    parts.append(paint(cp.finished_tray(), "#45c4b4", "finished_tray"))

    box = enclosure()
    box.label = "enclosure"
    parts.append(box)
    return bd.Compound(children=parts, label="forge_mini")


if __name__ == "__main__":
    cell()
