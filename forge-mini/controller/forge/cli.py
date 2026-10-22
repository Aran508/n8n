"""Command line: forge budget | ik | plan | export | run"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

from .budget import report
from .config import load_cell
from .kinematics import Unreachable, inverse
from .planner import PlanError, Planner


def load_recipe(path: str) -> dict:
    return yaml.safe_load(Path(path).read_text())


def plan_file(cell, path: str):
    recipe = load_recipe(path)
    planner = Planner(cell, recipe.get("arms"))
    initial = {name: list(st.joints) for name, st in planner.state.items()}
    timeline = planner.plan(recipe)
    timeline.initial = initial
    return recipe, timeline


def export_payload(cell, recipe: dict, timeline) -> dict:
    """Everything the 3D viewer needs: geometry, layout and the timed motion."""
    return {
        "initial": getattr(timeline, "initial", {}),
        "product": recipe.get("product"),
        "cell": {
            "links": cell.raw["links"],
            "arms": {n: {"base": a.base, "yaw": a.yaw, "tool": a.tool, "role": a.role}
                     for n, a in cell.arms.items() if n in (recipe.get("arms") or cell.arms)},
            "zone": cell.zone,
            "locations": cell.raw["locations"],
        },
        **timeline.to_dict(),
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="forge")
    p.add_argument("--cell", default=None, help="path to cell.yaml")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("budget", help="reach and servo torque check")
    ik = sub.add_parser("ik", help="joint + servo angles for one target")
    ik.add_argument("arm")
    ik.add_argument("x", type=float)
    ik.add_argument("y", type=float)
    ik.add_argument("z", type=float)
    pl = sub.add_parser("plan", help="validate a recipe in the digital twin")
    pl.add_argument("recipe")
    ex = sub.add_parser("export", help="plan and write the timeline JSON for the 3D viewer")
    ex.add_argument("recipe")
    ex.add_argument("-o", "--out", required=True)
    run = sub.add_parser("run", help="plan, then play on the real machine")
    run.add_argument("recipe")
    run.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0")
    run.add_argument("--camera", type=int, default=None, help="camera index for inspections")
    args = p.parse_args(argv)

    cell = load_cell(args.cell) if args.cell else load_cell()

    if args.cmd == "budget":
        print(report(cell))
        return 0

    if args.cmd == "ik":
        arm = cell.arms[args.arm]
        try:
            joints = inverse(cell, arm, (args.x, args.y, args.z), cell.motion["tool_pitch"])
        except Unreachable as e:
            print(f"unreachable: {e}")
            return 1
        print("joints:", joints)
        print("servos:", cell.joints_to_servos(arm, joints))
        return 0

    try:
        recipe, timeline = plan_file(cell, args.recipe)
    except PlanError as e:
        print(f"PLAN REJECTED: {e}", file=sys.stderr)
        return 2

    if args.cmd == "plan":
        steps = [e for e in timeline.events if e.type == "step"]
        print(f"OK: {recipe.get('product')} plans cleanly, {len(steps)} steps, "
              f"{len(timeline.segments)} moves, {timeline.duration:.1f}s")
        for e in steps:
            print(f"  {e.t:6.2f}s  {e.data['index']:>2}. {e.data['label']}")
        return 0

    if args.cmd == "export":
        Path(args.out).write_text(json.dumps(export_payload(cell, recipe, timeline)))
        print(f"wrote {args.out} ({timeline.duration:.1f}s)")
        return 0

    if args.cmd == "run":
        from .executor import SerialLink, play

        inspector = None
        if args.camera is not None:
            from .vision import Camera, make_inspector

            inspector = make_inspector(Camera(args.camera))
        link = SerialLink(args.port)
        try:
            play(cell, timeline, link, inspector)
        finally:
            link.send({"cmd": "aux", "ch": "pump", "on": False})
            link.close()
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
