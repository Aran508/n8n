# Forge Mini

A desktop micro-factory prototype: four small servo arms around one work plate, fed by a conveyor, with Claude (or any MCP client) driving it.

```
        arm1 Loader ●───────────● arm2 Holder
                     ╲         ╱
  conveyor ══▶ input  [ fixture ]  output tray
                     ╱         ╲
        arm3 Tool   ●───────────● arm4 Inspect
```

## What's here

| Path | What it is |
|---|---|
| `sim/forge-mini-sim.html` | Interactive 3D simulation of the four-week build. Open it in any browser. |
| `docs/BUILD_GUIDE.md` | Week-by-week build plan, with what to check in the simulator first |
| `docs/ARM_DESIGN.md` | Link lengths, joint conventions, IK maths, reach, torque check, bill of materials |
| `controller/` | Python: kinematics, recipe planner, hardware executor, vision, MCP server |
| `controller/cell.yaml` | The whole cell as data: link lengths, arm positions, servo map, trims, locations |
| `controller/recipes/` | Recipes for each week, plus two the planner is expected to reject |
| `firmware/forge_esp32/` | ESP32 firmware: smooth servo motion on 2× PCA9685, pump, conveyor, e-stop, watchdog |

## Quick start (no hardware needed)

```
cd controller
pip install -e ".[dev]"
forge budget                                   # reach + servo torque check
forge plan recipes/led_keychain.yaml           # validate in the digital twin
forge export recipes/led_keychain.yaml -o k.json   # open k.json in the simulator
pytest                                         # 27 tests
```

To rebuild the simulator after editing recipes or `cell.yaml`, run `python sim/build.py` from this folder.

## With the machine

```
pip install -e ".[hardware,vision,mcp]"
forge run recipes/led_keychain.yaml --port /dev/ttyUSB0 --camera 0
FORGE_PORT=/dev/ttyUSB0 python -m forge.mcp_server   # let Claude drive it
```

## How it stays safe

The AI only writes recipes. Each recipe is planned in full, with reach, joint limits and zone capacity checked for every 20 mm waypoint, before any servo moves. Running needs explicit confirmation. Below that, the ESP32 has a watchdog, and the e-stop cuts servo power in hardware.
