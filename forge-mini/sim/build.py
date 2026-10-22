"""Rebuild forge-mini-sim.html from the recipes, using the real planner.

    cd forge-mini && python sim/build.py
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "controller"))

from forge.cli import export_payload, plan_file  # noqa: E402
from forge.config import load_cell  # noqa: E402
from forge.planner import PlanError  # noqa: E402

RECIPES = HERE.parent / "controller" / "recipes"


def export(cell, name):
    recipe, timeline = plan_file(cell, RECIPES / f"{name}.yaml")
    return export_payload(cell, recipe, timeline)


def rejection(cell, name):
    try:
        plan_file(cell, RECIPES / f"{name}.yaml")
    except PlanError as e:
        return str(e)
    raise SystemExit(f"{name} was expected to be rejected by the planner")


def main():
    cell = load_cell()
    data = {
        "week1": export(cell, "week1_single_arm"),
        "week2": export(cell, "week2_four_arms"),
        "week3": export(cell, "led_keychain"),
        "conflict_error": rejection(cell, "week2_zone_conflict"),
        "draft_error": rejection(cell, "ai_first_draft"),
    }
    blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    out = HERE / "forge-mini-sim.html"
    out.write_text((HERE / "template.html").read_text().replace("__FORGE_DATA__", blob))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
