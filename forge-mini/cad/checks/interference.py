"""Interference and tool-point check on the saved cell (park pose).

Reports every pair of leaf parts from different top-level assemblies whose solids overlap by
more than 1 mm^3, plus overlaps inside each arm, and checks that each mounted tool's tip sits
where the planner puts it (park flange point minus tool length)."""

import itertools
import sys
from pathlib import Path

from cadgen import read_scene
from cadgen.geometry import overlap_volume

ROOT = Path(__file__).resolve().parent.parent
scene = read_scene(str(ROOT / "STEP" / "cell.step"))
ALLOWED = 1.0  # mm^3

leaves = list(scene.leaves())


def top(occ):
    parts = occ.ref.split("#")[-1].split(".")
    return parts[1] if len(parts) > 1 else parts[0]


shapes = {}
for occ in leaves:
    (sel,) = scene.resolve(occ.ref)
    shp = sel.shape()
    shapes[occ.ref] = (occ, top(occ), shp, shp.bounding_box(), shp.solids())

bad = []
items = list(shapes.values())
for (a, ta, sa, ba, solids_a), (b, tb, sb, bb, solids_b) in itertools.combinations(items, 2):
    if ta == tb and not (a.label or "").startswith("arm"):
        continue
    if not ba.overlaps(bb, tolerance=0):
        continue
    vol = sum(overlap_volume(x, y) for x in solids_a for y in solids_b)
    if vol > ALLOWED:
        bad.append((vol, a.label, b.label))

for vol, la, lb in sorted(bad, reverse=True):
    print(f"overlap {vol:8.1f} mm^3  {la}  x  {lb}")
print(f"{len(items)} parts, {len(bad)} overlapping pairs over {ALLOWED} mm^3")

# tool tips at the park pose: flange at (+-40, +-40, 60), tip = flange - tool length
EXPECT = {"arm1_tool_tweezers": (-40, 40, 60 - 44), "arm2_tool_iron_B": (40, 40, 60 - 52),
          "arm3_tool_vacuum": (-40, -40, 60 - 34), "arm4_tool_iron": (40, -40, 60 - 52)}
tip_ok = True
for label, (x, y, z) in EXPECT.items():
    (sel,) = scene.resolve("#" + label)
    bb = sel.shape().bounding_box()
    zmin = bb.min.Z
    print(f"{label:22s} lowest point z = {zmin:6.2f} (planner {z})")
    tip_ok &= abs(zmin - z) < 0.2
sys.exit(0 if (tip_ok and not bad) else 1)
