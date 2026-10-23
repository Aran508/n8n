"""MCP server so Claude (or any MCP client) can drive Forge Mini.

    FORGE_PORT=/dev/ttyUSB0 python -m forge.mcp_server     # real machine
    python -m forge.mcp_server                             # simulation only

Safety model: the model only ever submits recipes. Every recipe goes through
the planner (reach, joint limits, zone capacity) before anything moves, and
`run_recipe` needs an explicit confirm=True. The ESP32 e-stop and watchdog
sit below all of this and cannot be overridden from here.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml
try:  # mcp >= 2
    from mcp.server.mcpserver import MCPServer
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as MCPServer

from .config import load_cell
from .kinematics import Unreachable, inverse, max_reach
from .planner import PlanError, Planner

RECIPES = Path(__file__).resolve().parent.parent / "recipes"
cell = load_cell()
mcp = MCPServer("forge-mini")
_link = None


def _machine():
    global _link
    port = os.environ.get("FORGE_PORT")
    if not port:
        return None
    if _link is None:
        from .executor import SerialLink

        _link = SerialLink(port)
    return _link


@mcp.tool()
def describe_cell() -> dict:
    """Arms and their roles, tool racks, bits, named locations, shared zones and reach limits.

    Every arm starts with no tool. Use tool_change / bit_change steps; an arm can
    only use the rack listed for it.
    """
    return {
        "arms": {n: {"role": a.role, "base_xy": a.base} for n, a in cell.arms.items()},
        "tools": {n: {"type": t.type, "rack": t.rack, "reachable_by": cell.racks[t.rack]["arms"], "length_mm": t.length}
                  for n, t in cell.tools.items()},
        "bits": {n: {"kind": b.kind, "dia_mm": b.dia, "reachable_by": cell.bit_block["arms"]} for n, b in cell.bits.items()},
        "tool_actions": {"pick/place": "gripper, tweezers or vacuum", "hold": "clamp or gripper",
                         "machine": "spindle + bit", "drive": "spindle + hex bit (screws)",
                         "solder": "iron (list of joint locations)",
                         "draw": "pen or glue (path of [dx, dy] offsets)", "probe": "probe",
                         "flip": "no arm: the flip fixture turns the part over so arms can work on its underside "
                                 "(release clamps first; a point (x, y, z) moves to (x, -y, 2*axis_z - z))"},
        "flipper": cell.raw["flipper"],
        "scheduling": "steps run as early as possible (schedule: auto, the default): write them in a sensible "
                      "order and the planner overlaps work by different arms; set schedule: sequential to disable",
        "locations": cell.raw["locations"],
        "zones": cell.zones,
        "max_wrist_reach_mm": round(max_reach(cell)),
        "actions": sorted(Planner.ARM_ACTIONS | Planner.CELL_ACTIONS | {"parallel"}),
        "units": "mm, world origin at the centre of the work plate, Z up",
    }


@mcp.tool()
def check_reach(arm: str, x: float, y: float, z: float, tool: str | None = None) -> dict:
    """Can this arm put the tip of `tool` (or its bare coupler) at (x, y, z) pointing down?"""
    l3 = cell.links.coupler + (cell.tools[tool].length if tool else 0)
    try:
        joints = inverse(cell, cell.arms[arm], (x, y, z), cell.motion["tool_pitch"], 0, l3)
        return {"reachable": True, "joints": joints}
    except Unreachable as e:
        return {"reachable": False, "reason": str(e)}


@mcp.tool()
def list_recipes() -> list[str]:
    """Saved recipe names."""
    return sorted(p.stem for p in RECIPES.glob("*.yaml"))


@mcp.tool()
def get_recipe(name: str) -> str:
    """YAML source of a saved recipe."""
    return (RECIPES / f"{Path(name).stem}.yaml").read_text()


@mcp.tool()
def plan_recipe(recipe_yaml: str) -> dict:
    """Validate a recipe in the digital twin without moving anything.

    Returns the step list with start times, or the exact step and reason it fails.
    Fix the recipe and call again until ok is true.
    """
    try:
        recipe = yaml.safe_load(recipe_yaml)
        tl = Planner(cell, recipe.get("arms")).plan(recipe)
    except (PlanError, yaml.YAMLError, KeyError, TypeError) as e:
        return {"ok": False, "error": str(e)}
    steps = sorted(({"t": e.t, "arm": e.data.get("arm"), "step": e.data["label"]} for e in tl.events if e.type == "step"),
                   key=lambda s: s["t"])
    return {"ok": True, "duration_s": tl.duration, "moves": len(tl.segments), "busy_pct": tl.stats["busy_pct"],
            "steps": steps}


@mcp.tool()
def save_recipe(name: str, recipe_yaml: str) -> dict:
    """Save a recipe after it plans cleanly."""
    result = plan_recipe(recipe_yaml)
    if result["ok"]:
        (RECIPES / f"{Path(name).stem}.yaml").write_text(recipe_yaml)
    return result


@mcp.tool()
def run_recipe(name: str, confirm: bool = False) -> dict:
    """Run a saved recipe on the machine. Ask the user first, then pass confirm=True."""
    if not confirm:
        return {"ok": False, "error": "not confirmed: show the plan to the user and pass confirm=True"}
    recipe = yaml.safe_load(get_recipe(name))
    try:
        tl = Planner(cell, recipe.get("arms")).plan(recipe)
    except PlanError as e:
        return {"ok": False, "error": str(e)}
    link = _machine()
    if link is None:
        return {"ok": True, "simulated": True, "duration_s": tl.duration,
                "note": "FORGE_PORT not set, so nothing moved"}
    from .executor import MachineError, play

    log: list[str] = []
    try:
        play(cell, tl, link, log=log.append)
    except MachineError as e:
        return {"ok": False, "error": str(e), "log": log}
    return {"ok": True, "duration_s": tl.duration, "log": log}


@mcp.tool()
def machine_status() -> dict:
    """Live servo angles, e-stop state and IR sensor from the ESP32."""
    link = _machine()
    if link is None:
        return {"connected": False}
    return {"connected": True, **link.send({"cmd": "status"})}


@mcp.tool()
def stop() -> dict:
    """Relax every servo and switch every tool off immediately."""
    link = _machine()
    if link is None:
        return {"connected": False}
    link.send({"cmd": "relax"})
    return {"ok": True}


if __name__ == "__main__":
    mcp.run()
