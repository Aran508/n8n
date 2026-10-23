# Build guide: four weeks, simulation first

Every week begins in the simulator. The recipe has to plan cleanly and look right in 3D before anything is built or powered on.
Open `sim/forge-mini-sim.html` in a browser to watch each week.

```
cd forge-mini/controller
pip install -e ".[dev]"
forge plan recipes/<recipe>.yaml                 # digital-twin check
forge export recipes/<recipe>.yaml -o my.json    # load my.json in the viewer
```

There is no conveyor. You load parts by hand: the plate goes in the **kit tray** at the front, and pins go in the **pin cup** at the back. The finished part ends up in the **finished tray** at the back.

---

## Week 1: one arm, first tool changes

**Goal:** one arm goes where the code says, and its coupler locks, identifies and releases a tool every time.

**Build**
- One arm: MG996R base, shoulder and elbow; MG90S wrist.
- One coupler on the wrist and a coupler plate on each tool (see `docs/TOOL_CHANGER.md`).
- The left rack with gripper, vacuum, pen and probe.
- ESP32 + PCA9685 at 0x40, a 6 V servo supply and the e-stop.

**Do**
1. Flash `firmware/forge_esp32` (Arduino IDE, with the *Adafruit PWM Servo Driver* and *ArduinoJson* libraries).
2. Fit the servo horns at 90°: `{"cmd":"move","arm":1,"servo":[90,90,90,90,20],"ms":500}`.
3. Plug each tool onto the coupler by hand and send `{"cmd":"status"}`. Check that `tool_ohms` shows that tool's ID resistor.
4. Run `recipes/week1_single_arm.yaml`: calibration sweep, gripper on, block move, pen on, 40 mm square.
5. Measure the square with calipers. If it is too small, skewed or rotated, adjust `trims.arm1` in `cell.yaml`.

**Done when** 20 tool changes in a row pass the ID check and the square measures 40.0 ± 0.5 mm.

---

## Week 2: four arms, two racks

**Goal:** four arms change tools and share the plate without touching.

**Build**
- Three more arms. Second PCA9685 at 0x41 (bridge the A0 pad).
- The right rack with gripper, clamp, spindle and glue.
- The bit block with its stripper bar, and the five bits.

**The anti-collision rules** are shared zones: each rack and each tray admits one arm at a time, and the fixture admits three. On top of that, every pair of tools stays 20 mm apart. The auto scheduler simply waits until there is room. A recipe with `schedule: sequential` runs exactly as written, so the planner rejects it instead:

```
$ forge plan recipes/week2_zone_conflict.yaml
PLAN REJECTED: zone conflict in rack_L at t=0.66s: arm1, arm3 are inside at once (capacity 1).
Put these steps in sequence instead of in parallel.
```

**Simulation:** all four calibrate together, then tool up in pairs (one arm per rack). Arm 4 loads a drill from the bit block, arm 3 probes a block, arm 1 moves it and arm 2 clamps it. Then every tool goes home.

**Done when** `week2_four_arms.yaml` runs 5 times with no contact and every tool ends in its own slot.

---

## Week 3: first product, a doweled bracket

**Goal:** you load a printed plate and two pins, and the cell does the rest.

The plate is printed with **1.8 mm pilot holes**. The cell reams them to an exact **2.0 mm**, chamfers them, presses two 2 mm dowel pins in, measures the pins, marks the part and unloads it.

| # | Arm | Tool | Action |
|---|---|---|---|
| 1–2 | all | – | tool up: arm3 vacuum, arm2 clamp, arm1 gripper, arm4 spindle + 2.0 mm reamer |
| 3–4 | arm3 Loader | vacuum | plate from kit tray to fixture |
| 5 | arm2 Holder | clamp | hold the plate down |
| 6 | arm4 Machinist | spindle + ream_2_0 | ream both holes, 5 mm deep at 2 mm/s |
| 7–8 | arm4 | spindle + csink_90 | swap bit, chamfer both holes |
| 9 | arm2 | clamp | release (the clamp is only needed while cutting) |
| 10–13 | arm1 Assembler | gripper | press pin 1, then pin 2 (spindle goes home at the same time) |
| 14–15 | arm3 | probe | swap tool, measure both pins: 9.0 ± 0.3 mm |
| 16–17 | arm3 | pen | swap tool, mark the part |
| 18 | camera | – | overhead camera check |
| 19–21 | arm1 | gripper | bracket to the finished tray; everyone returns their tools |

About 84 s with the auto scheduler (104 s one step at a time), 6 tool changes and 2 bit changes.

**Done when** 10 brackets in a row probe in tolerance and a 2.0 mm gauge pin slides into a spare reamed hole.

Then move on to the projects in `docs/PROJECTS.md`: micro-drone, desk robot head and LED bar. They add the tweezers and soldering iron, and show the scheduler keeping all four arms busy.

---

## Week 4: Claude takes the controls

**Goal:** you describe the part, and Claude picks the tools, writes the recipe, checks it and runs it.

**Set up on the Pi**
```
pip install -e "controller[hardware,mcp]"
```
Claude Desktop / Claude Code MCP config:
```json
{
  "mcpServers": {
    "forge-mini": {
      "command": "python",
      "args": ["-m", "forge.mcp_server"],
      "env": { "FORGE_PORT": "/dev/ttyUSB0" }
    }
  }
}
```
Without `FORGE_PORT` the server runs in simulation-only mode, which is a safe way to try it first.

**Tools Claude gets:** `describe_cell` (arms, tools, racks, bits, zones), `check_reach`, `list_recipes`, `get_recipe`, `plan_recipe`, `save_recipe`, `run_recipe` (needs `confirm=true`), `machine_status`, `stop`.

**What a session looks like.** Claude's first draft (`recipes/ai_first_draft.yaml`) asks arm 2 to fetch the pen. The planner answers:

```
step 15 (Probe out, pen in): arm2 cannot reach rack_L, where pen lives; use arm1 or arm3
```

Claude gives the marking to arm 3, the plan comes back clean, Claude asks you, and the run starts. Other errors are just as specific, for example:
- `arm3 needs a gripper/vacuum tool to pick but holds no tool; add tool_change: {arm: arm3, tool: gripper_A} first`
- `pen is on arm1; return it first with tool_change: {arm: arm1, tool: none}`
- `arm4 has the spindle but no bit; add bit_change: {arm: arm4, bit: ...}`

**Safety layers, top to bottom**
1. The model can only submit recipes. It never sends raw servo commands.
2. The planner tracks every tool, bit and part, checks reach for every 20 mm waypoint with the mounted tool's length, and enforces the zones.
3. `run_recipe` refuses to run without explicit confirmation.
4. Every tool change is checked against the tool's ID resistor.
5. The ESP32 watchdog stops arms and spindles if the Pi goes quiet for 1.5 s.
6. The hardware e-stop cuts servo power, and software cannot override it.

---

## After the prototype

- Add more tools: a soldering pen (with a heat-resistant coupler plate), a screwdriver with torque sensing, or a laser pointer for alignment.
- Swap hobby servos for smart servos with position feedback (STS3215 or Dynamixel) for stiffer, more accurate machining.
- Move to ROS 2 + MoveIt 2 for real collision meshes instead of zones.
- Scale the links. Everything is a parameter in `cell.yaml`, and `forge budget` tells you which servos you need.
