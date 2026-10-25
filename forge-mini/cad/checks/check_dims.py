"""Fails if src/lib/dims.py has drifted from controller/cell.yaml. Plain Python + PyYAML:

    python checks/check_dims.py
"""

import importlib.util
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
cell = yaml.safe_load((ROOT.parent / "controller" / "cell.yaml").read_text())
spec = importlib.util.spec_from_file_location("dims", ROOT / "src" / "lib" / "dims.py")
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)

errors = []


def same(what, a, b):
    if abs(float(a) - float(b)) > 1e-6:
        errors.append(f"{what}: dims.py {a} != cell.yaml {b}")


for k, v in (("h0", d.H0), ("L1", d.L1), ("L2", d.L2), ("coupler", d.COUPLER)):
    same(f"links.{k}", v, cell["links"][k])
for name, (bx, by, yaw, _) in d.ARMS.items():
    a = cell["arms"][name]
    same(f"{name}.base.x", bx, a["base"][0]); same(f"{name}.base.y", by, a["base"][1]); same(f"{name}.yaw", yaw, a["yaw"])
for name, (kind, rack, x, y, length) in d.TOOLS.items():
    t = cell["tools"][name]
    same(f"{name}.slot.x", x, t["slot"][0]); same(f"{name}.slot.y", y, t["slot"][1]); same(f"{name}.length", length, t["length"])
    if rack != t["rack"]:
        errors.append(f"{name}.rack: {rack} != {t['rack']}")
for name, (kind, dia, stickout, y) in d.BITS.items():
    b = cell["bits"][name]
    same(f"{name}.dia", dia, b["dia"]); same(f"{name}.stickout", stickout, b["stickout"]); same(f"{name}.slot.y", y, b["slot"][1])
    same(f"{name}.slot.x", d.BIT_BLOCK_X, b["slot"][0])
for r in cell["racks"].values():
    same("rack z", d.RACK_Z, r["z"])
same("bit_block.z", d.BIT_TOP_Z, cell["bit_block"]["z"])
same("flipper.axis_z", d.FLIP_AXIS_Z, cell["flipper"]["axis_z"])
for name, j in (("base", "base"), ("shoulder", "shoulder"), ("elbow", "elbow"), ("wrist", "wrist")):
    lo, hi = d.LIMITS[name]
    same(f"{name}.min", lo, cell["servo_map"][j]["min"]); same(f"{name}.max", hi, cell["servo_map"][j]["max"])
loc = cell["locations"]
for name, (x, y) in {**d.CUPS, "kit.plate": d.KIT_TRAY, "side.part": d.SIDE_PART, "finished": d.FINISHED}.items():
    same(f"{name}.x", x, loc[name][0]); same(f"{name}.y", y, loc[name][1])
slots_y = sorted({t["slot"][1] for t in cell["tools"].values() if t["rack"] == "rack_L"}, reverse=True)
if list(d.RACK_SLOTS_Y) != slots_y:
    errors.append(f"rack slots {d.RACK_SLOTS_Y} != {slots_y}")
if 2 * d.PLATE_R >= d.SLOT_PITCH:
    errors.append(f"tool plates ({2 * d.PLATE_R} mm) don't fit the {d.SLOT_PITCH} mm slot pitch")

print("\n".join(errors) or "dims.py matches cell.yaml")
sys.exit(1 if errors else 0)
