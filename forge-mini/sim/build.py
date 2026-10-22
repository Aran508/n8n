"""Rebuild forge-mini-sim.html from the recipes, using the real planner.

Every job is planned twice: with the auto scheduler (arms work side by side)
and one step at a time, so the viewer can play both and show the difference.

    cd forge-mini && python sim/build.py
"""

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "controller"))

from forge.cli import export_payload, load_recipe  # noqa: E402
from forge.config import load_cell  # noqa: E402
from forge.planner import PlanError, Planner  # noqa: E402

RECIPES = HERE.parent / "controller" / "recipes"
JOBS = {
    "week1": "week1_single_arm",
    "week2": "week2_four_arms",
    "week3": "doweled_bracket",
    "drone": "micro_drone",
    "bot": "desk_bot_head",
    "led": "led_bar",
}


def export(cell, name, schedule):
    recipe = {**load_recipe(RECIPES / f"{name}.yaml"), "schedule": schedule}
    timeline = Planner(cell, recipe.get("arms")).plan(recipe)
    payload = export_payload(cell, recipe, timeline)
    payload["steps"] = sum(1 for e in timeline.events if e.type == "step")
    return payload


def rejection(cell, name):
    recipe = load_recipe(RECIPES / f"{name}.yaml")
    try:
        Planner(cell, recipe.get("arms")).plan(recipe)
    except PlanError as e:
        return str(e)
    raise SystemExit(f"{name} was expected to be rejected by the planner")


def main():
    cell = load_cell()
    data = {}
    for key, name in JOBS.items():
        data[key] = export(cell, name, "auto")
        data[key + "_seq"] = export(cell, name, "sequential")
        a, s = data[key]["duration"], data[key + "_seq"]["duration"]
        print(f"{name:<18} auto {a:6.1f}s   one-at-a-time {s:6.1f}s   saves {100 * (1 - a / s):3.0f}%")
    data["conflict_error"] = rejection(cell, "week2_zone_conflict")
    data["draft_error"] = rejection(cell, "ai_first_draft")
    blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    out = HERE / "forge-mini-sim.html"
    out.write_text((HERE / "template.html").read_text().replace("__FORGE_DATA__", blob))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
