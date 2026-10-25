"""Generate smartwatch.yaml: the perfboard ESP32-C3 smartwatch, planned for the cell.

The layout follows the hand-built watch: a 40 x 40 mm perfboard with the
ESP32-C3 SuperMini at the top, the USB-C charger module beside it, the OLED
header in the middle and three 12 mm buttons along the bottom edge. Every
through-hole leg is soldered on the underside after a flip; the hand-routed
wires become drag-soldered tracks. Edit the layout here and re-run:

    python recipes/make_smartwatch.py
"""

from pathlib import Path

import yaml

PITCH = 2.54
TOP = 3.0                # board top surface on the fixture
AXIS_Z = 10.0            # flip axis (cell.yaml flipper.axis_z)
BOARD_T = 1.6
UNDER = 2 * AXIS_Z - (TOP - BOARD_T)  # underside surface height after a flip (18.6)


def under(x, y):
    """Where a leg at (x, y) on the top ends up after the board is flipped."""
    return [round(x, 2), round(-y, 2), round(UNDER + 0.6, 2)]


loc, joints = {}, {}


def part(name, x, y, z, legs):
    loc[f"watch.{name}"] = [x, y, z]
    for i, (lx, ly) in enumerate(legs, 1):
        key = f"watch.u.{name}{i}"
        loc[key] = under(lx, ly)
        joints.setdefault(name, []).append(key)


loc["watch.board"] = [0, 0, TOP]
esp_x = [-3 - 3.5 * PITCH + k * PITCH for k in range(8)]
part("esp", -3, 9, 8, [(x, 9 - 3 * PITCH) for x in esp_x] + [(x, 9 + 3 * PITCH) for x in esp_x])
part("hdr", 0, -3, 11.5, [(-1.5 * PITCH + k * PITCH, -3) for k in range(4)])
part("r1", -14, -3, 5, [(-14 - 1.5 * PITCH, -3), (-14 + 1.5 * PITCH, -3)])
part("q1", 11, -3, 7, [(11 - PITCH, -3), (11, -3), (11 + PITCH, -3)])
part("jst", -14, -8, 7, [(-14 - PITCH / 2, -8), (-14 + PITCH / 2, -8)])
for i, bx in enumerate((-12, 0, 12), 1):
    part(f"b{i}", bx, -14, 8, [(bx + dx, -14 + dy) for dx in (-1.5 * PITCH, 1.5 * PITCH) for dy in (-PITCH, PITCH)])
# charger module (castellated pads, soldered on top)
loc["watch.chg"] = [14, 9, 4.6]
chg_pads = []
for i, (px, py) in enumerate([(9.5, 4), (18.5, 4), (9.5, 14), (18.5, 14)], 1):
    loc[f"watch.chg{i}"] = [px, py, TOP + 0.4]
    chg_pads.append(f"watch.chg{i}")
loc["watch.oled"] = [0, 10.5, 14.5]
# after the second flip: back cover on standoffs over the battery
loc["watch.back"] = [0, 0, round(UNDER + 8, 2)]
loc["watch.s1"] = [-16, 16, round(UNDER + 8, 2)]
loc["watch.s2"] = [16, -16, round(UNDER + 8, 2)]
# kit: the loader drawer brings one kit per watch
loc["kit.board"] = [6, -108, 4]
loc["kit.oled"] = [-22, -100, 4]

# drag-soldered tracks on the underside ([dx, dy] from watch.u0); representative
# wiring - generate these from your own schematic's netlist
loc["watch.u0"] = [0, 0, round(UNDER + 0.6, 2)]
f = lambda p: [round(p[0], 2), round(-p[1], 2)]  # top-side coords -> underside offsets
tracks = [
    ("GND rail along the buttons", [f((-15.8, -16.5)), f((15.8, -16.5))]),
    ("OLED VCC/GND to the ESP 3V3/GND row", [f((-3.8, -3)), f((-3.8, 1.4)), f((-5.2, 1.4))]),
    ("OLED SDA to GPIO", [f((1.3, -3)), f((1.3, 1.4)), f((1.6, 1.4))]),
    ("OLED SCL to GPIO", [f((3.8, -3)), f((4.1, 1.4))]),
    ("button 1 to GPIO", [f((-8.2, -11.5)), f((-8.2, 1.4)), f((-9.0, 1.4))]),
    ("button 2 to GPIO", [f((3.8, -11.5)), f((6.5, -6)), f((6.5, 1.4))]),
    ("pull-up and MOSFET gate", [f((-10.2, -3)), f((8.5, -3))]),
]

steps = [
    {"tool_change": {"arm": "arm3", "tool": "vacuum"}},
    {"tool_change": {"arm": "arm1", "tool": "tweezers"}},
    {"tool_change": {"arm": "arm2", "tool": "gripper_B"}},
    {"tool_change": {"arm": "arm4", "tool": "iron"}},
    {"pick": {"arm": "arm3", "item": "board", "from": "kit.board"}, "note": "Perfboard from the kit drawer into the flip fixture"},
    {"place": {"arm": "arm3", "to": "watch.board"}},
    {"pick": {"arm": "arm3", "item": "esp", "from": "side.part"}, "note": "ESP32-C3 SuperMini, headers pre-soldered"},
    {"place": {"arm": "arm3", "to": "watch.esp", "into": "board", "press": 1}},
]
for i in (1, 2, 3):
    steps += [{"pick": {"arm": "arm2", "item": f"b{i}", "from": "cup_a"}, **({"note": "Buttons go in along the bottom edge"} if i == 1 else {})},
              {"place": {"arm": "arm2", "to": f"watch.b{i}", "into": "board", "press": 1}}]
for name, note in (("r1", "Small parts with tweezers: 10k pull-up"), ("q1", "P-MOSFET power switch"),
                   ("hdr", "4-pin socket for the OLED"), ("jst", "Vertical JST socket for the battery")):
    steps += [{"pick": {"arm": "arm1", "item": name, "from": "cup_b"}, "note": note},
              {"place": {"arm": "arm1", "to": f"watch.{name}", "into": "board", "press": 1}}]
steps += [
    {"pick": {"arm": "arm3", "item": "chg", "from": "cup_c"}, "note": "USB-C charger module, castellated pads"},
    {"place": {"arm": "arm3", "to": "watch.chg", "into": "board"}},
    {"solder": {"arm": "arm4", "at": chg_pads, "dwell": 0.6}, "note": "Iron solders the charger's 4 pads on top"},
    {"tool_change": {"arm": "arm2", "tool": "iron_B"}, "note": "Holder swaps its gripper for the second iron"},
    {"flip": {"at": "watch"}, "note": "Flip: legs point up"},
]
# two irons work the underside at once: arm4 takes the top half, arm2 the bottom
iron_of = {"esp": "arm4", "q1": "arm4", "b3": "arm4", "hdr": "arm2", "r1": "arm2", "jst": "arm2", "b1": "arm2", "b2": "arm2"}
for name in ["esp", "b1", "q1", "r1", "b3", "jst", "b2", "hdr"]:
    steps.append({"solder": {"arm": iron_of[name], "at": joints[name], "dwell": 0.5},
                  "note": f"Underside joints: {name} ({len(joints[name])} legs, {iron_of[name]})"})
track_arm = ["arm2", "arm4", "arm4", "arm4", "arm2", "arm2", "arm2"]
for (label, path), a in zip(tracks, track_arm):
    steps.append({"trace": {"arm": a, "at": "watch.u0", "path": path}, "note": f"Track: {label} ({a})"})
steps += [
    {"flip": {"at": "watch"}, "note": "Flip back"},
    {"pick": {"arm": "arm3", "item": "oled", "from": "kit.oled"}, "note": "OLED module presses into its socket"},
    {"place": {"arm": "arm3", "to": "watch.oled", "into": "board", "press": 1}},
    {"flash": {"at": "watch", "firmware": "watch_firmware.bin"}, "note": "USB-C dock: flash firmware and self-test"},
    {"inspect": {"at": "watch.oled", "check": "watch_face_shows"}, "note": "Camera checks the watch face"},
    {"operator": {"message": "Plug the battery into the JST socket and lay it on the back (foam tape). Press START.", "seconds": 30},
     "note": "You: plug in the battery (30 s)"},
    {"flip": {"at": "watch"}, "note": "Flip to fit the back cover"},
    {"tool_change": {"arm": "arm4", "tool": "spindle"}},
    {"bit_change": {"arm": "arm4", "bit": "hex_1_5"}},
    {"pick": {"arm": "arm3", "item": "back", "from": "kit.board"}, "note": "Back cover (second perfboard) onto its standoffs"},
    {"place": {"arm": "arm3", "to": "watch.back", "into": "board"}},
    {"pick": {"arm": "arm1", "item": "s1", "from": "cup_a"}, "note": "Two M2 screws through the back cover"},
    {"place": {"arm": "arm1", "to": "watch.s1", "into": "board"}},
    {"pick": {"arm": "arm1", "item": "s2", "from": "cup_a"}},
    {"place": {"arm": "arm1", "to": "watch.s2", "into": "board"}},
    {"drive": {"arm": "arm4", "at": ["watch.s1", "watch.s2"], "depth": 2}, "note": "Drive both screws from below"},
    {"flip": {"at": "watch"}, "note": "Flip back, face up"},
    {"tool_change": {"arm": "arm1", "tool": "gripper_A"}},
    {"pick": {"arm": "arm1", "item": "board", "from": "watch.board"}, "note": "Finished watch to the tray"},
    {"place": {"arm": "arm1", "to": "finished"}},
    {"tool_change": {"arm": "arm1", "tool": "none"}},
    {"tool_change": {"arm": "arm2", "tool": "none"}},
    {"tool_change": {"arm": "arm3", "tool": "none"}},
    {"tool_change": {"arm": "arm4", "tool": "none"}},
]

items = {
    "back": {"at": "kit.board", "kind": "perfboard"},   # lies under the board in the same kit pocket
    "board": {"at": "kit.board", "kind": "perfboard"},
    "esp": {"at": "side.part", "kind": "esp32c3"},
    "chg": {"at": "cup_c", "kind": "charger"},
    "oled": {"at": "kit.oled", "kind": "oled_watch"},
    **{f"b{i}": {"at": "cup_a", "kind": "button"} for i in (1, 2, 3)},
    "r1": {"at": "cup_b", "kind": "resistor"},
    "q1": {"at": "cup_b", "kind": "mosfet"},
    "hdr": {"at": "cup_b", "kind": "header"},
    "jst": {"at": "cup_b", "kind": "jst"},
    "s1": {"at": "cup_a", "kind": "screw"},
    "s2": {"at": "cup_a", "kind": "screw"},
}

header = """# Project: perfboard smartwatch (ESP32-C3 SuperMini, USB-C charger, 0.96" OLED,
# 3 buttons, P-MOSFET power switch, LiPo). Generated by make_smartwatch.py -
# edit that file, not this one.
#
# The cell does the board work: place every part, solder 4 pads on top, flip,
# solder 39 legs and 7 drag-soldered tracks underneath with two irons at once, fit the OLED, flash
# and self-test the firmware over a USB-C dock, then (after you plug in the
# battery) fit the back cover with two screws from below.
# Left for a person: plugging in the battery (a flexible lead) and the strap.
"""
recipe = {"product": "smartwatch", "items": items, "locations": loc, "steps": steps}
out = Path(__file__).with_name("smartwatch.yaml")
out.write_text(header + yaml.safe_dump(recipe, sort_keys=False, width=120))
print(f"wrote {out}: {len(steps)} steps, {sum(len(v) for v in joints.values()) + len(chg_pads)} solder joints, {len(tracks)} tracks")
