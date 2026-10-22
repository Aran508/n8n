# Forge Mini

A desktop micro-factory prototype: four small servo arms around one work plate. Each arm has an automatic tool changer and shares two tool racks and a reamer and bit block with its neighbours. A scheduler keeps all four arms working at once. It builds small DIY electronics, such as a micro-drone, a desk robot head with an OLED face and a soldered LED bar, and Claude, or any MCP client, can drive it.

```
              back: pin cup · finished tray
     arm1 Assembler ●───────────● arm2 Holder
 left rack  |            [ fixture ]            |  right rack + bit block
 gripper    |                                   |  gripper, clamp,
 tweezers   |                                   |  spindle, iron, glue
 vacuum     |                                   |  reamers, drill,
 pen, probe |                                   |  countersink, hex
     arm3 Loader    ●───────────● arm4 Machinist
              front: kit tray (you load parts here)
```

## What's here

| Path | What it is |
|---|---|
| `sim/forge-mini-sim.html` | Interactive 3D simulation of the four-week build and the three projects. Open it in any browser. |
| `docs/BUILD_GUIDE.md` | Week-by-week build plan, with what to check in the simulator first |
| `docs/PROJECTS.md` | The drone, desk-bot and LED-bar projects, and how the scheduler keeps every arm busy |
| `docs/TOOL_CHANGER.md` | Coupler design, tool ID resistors, racks, bit block, adding a tool |
| `docs/ARM_DESIGN.md` | Link lengths, joint conventions, IK maths, servo budget, bill of materials |
| `controller/` | Python: kinematics, recipe planner, hardware executor, vision, MCP server |
| `controller/cell.yaml` | The whole cell as data: arms, tools, racks, bits, zones, locations |
| `controller/recipes/` | Recipes for each week, plus two the planner is expected to reject |
| `firmware/forge_esp32/` | ESP32 firmware: 24 servo channels, tool signals, tool ID, probe, e-stop, watchdog |

## Quick start (no hardware needed)

```
cd controller
pip install -e ".[dev]"
forge tools                                       # racks, tools and bits
forge budget                                      # reach, servo load per tool, reaming force
forge plan recipes/micro_drone.yaml               # validate, schedule, show arm usage
forge export recipes/micro_drone.yaml -o d.json   # open d.json in the simulator
pytest                                            # 47 tests
```

To rebuild the simulator after editing recipes or `cell.yaml`, run `python sim/build.py` from this folder.

## Recipes

A recipe lists steps. Arms start bare, so the recipe says which tools to use:

```yaml
steps:
  - tool_change: {arm: arm4, tool: spindle}
  - bit_change:  {arm: arm4, bit: ream_2_0}
  - machine:     {arm: arm4, at: [fixture.hole1, fixture.hole2], depth: 5}
  - tool_change: {arm: arm4, tool: none}      # returns the bit, then the spindle
```

Actions: `tool_change`, `bit_change`, `pick`, `place` (with optional `into` and `press`), `hold`, `release`, `machine`, `drive` (screws), `solder`, `draw`, `probe`, `inspect`, `move`, `park`, `calibrate`, `wait`.

You don't need to say what runs in parallel. With the default `schedule: auto`, every step starts as soon as its arm, its parts and tools, the shared zones and a 20 mm clearance between tools allow. That makes the projects 30–50% faster than running one step at a time (see `docs/PROJECTS.md`).

## With the machine

```
pip install -e ".[hardware,vision,mcp]"
forge run recipes/doweled_bracket.yaml --port /dev/ttyUSB0 --camera 0
FORGE_PORT=/dev/ttyUSB0 python -m forge.mcp_server   # let Claude drive it
```

## How it stays safe

The AI only writes recipes. Each recipe is planned in full before any servo moves: tool, bit and part positions, reach with the mounted tool's length for every 20 mm waypoint, capacity of the shared zones (racks, trays, fixture), and 20 mm clearance between every pair of tools, checked every 50 ms. Running needs explicit confirmation. Every tool change is verified by the tool's ID resistor. Below that, the ESP32 has a watchdog, and the e-stop cuts servo power in hardware.
