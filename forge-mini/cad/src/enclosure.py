"""The miniature robot cell around the 300 x 300 base: 2020 frame, clear acrylic, hinged door
with a reed-switch interlock, light curtain across the loading window, tower light, overhead
LED bar and camera, fume fan, operator console and floor marking (docs/ENCLOSURE.md)."""

from __future__ import annotations

import cadgen
from cadgen import build123d as bd
from cadgen import step

from lib import cell_parts as cp
from lib.geo import ALU, group, paint

DOOR_MATE = cadgen.revolute("door", parent="#enclosure_frame", child="#door",
                            origin=(cp.DOOR_HINGE[0], cp.DOOR_HINGE[1], 0), direction=(0, 0, -1), limits=(0, 110))
KINEMATICS = {"mates": [DOOR_MATE], "poses": {"closed": {"door": 0}, "open": {"door": 100}}}


def enclosure_shape() -> bd.Compound:
    door = group("door", [paint(cp.door(), "#cfe8ff", "door_pane", 0.22),
                          paint(cp.door_hardware(), "#2b2f36", "door_hardware")])
    curtain = cp.light_curtain()
    return group("enclosure", [
        paint(cp.frame(), ALU, "enclosure_frame"),
        paint(cp.panels(), "#cfe8ff", "acrylic_panels", 0.18),
        door,
        paint(cp.reed_switch(), "#d92b25", "door_reed_switch"),
        paint(curtain[0], "#1d1f22", "light_curtain_emitter"),
        paint(curtain[1], "#1d1f22", "light_curtain_receiver"),
        *[paint(s, c, label) for s, label, c in cp.tower_light()],
        *[paint(s, c, label) for s, label, c in cp.roof_gear()],
        *[paint(s, c, label) for s, label, c in cp.fan()],
        *[paint(s, c, label) for s, label, c in cp.console()],
        paint(cp.floor_marking(), "#f2c31e", "floor_marking"),
    ])


@step(out="../STEP/cell/enclosure.step", kinematics=KINEMATICS)
def enclosure():
    return enclosure_shape()


if __name__ == "__main__":
    enclosure()
