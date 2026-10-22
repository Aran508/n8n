# Build guide: four weeks, simulation first

Every week begins in the simulator. The recipe has to plan cleanly and look right in 3D before anything is built or powered on.
Open `sim/forge-mini-sim.html` in a browser to watch each week.

```
cd forge-mini/controller
pip install -e ".[dev]"
forge plan recipes/<recipe>.yaml        # digital-twin check
forge export recipes/<recipe>.yaml -o my.json   # load my.json in the viewer
```

---

## Week 1: one arm on the bench

**Goal:** one arm goes exactly where the code says.

**Build**
- Print the base turret and the three links (55 / 80 / 80 / 45 mm).
- Fit the servos: MG996R on base and shoulder, MG90S on the elbow, SG90 on wrist and gripper.
- ESP32 + PCA9685 at 0x40, a 6 V 10 A servo supply, and the e-stop in series with it.

**Do**
1. Flash `firmware/forge_esp32` (Arduino IDE, with the *Adafruit PWM Servo Driver* and *ArduinoJson* libraries).
2. In the serial monitor, send `{"cmd":"move","arm":1,"servo":[90,90,90,90,90],"ms":500}` and fit the horns at 90°.
3. Run `forge budget` to confirm reach and torque.
4. Run `recipes/week1_single_arm.yaml`: the calibration sweep, then block pick-and-place between pads A and B.
5. Measure where the block lands. Write each joint's error into `trims.arm1` in `cell.yaml`.

**Simulation:** arm 1 rises to its ready pose and sweeps each joint ±20°, then moves the block A → B → A.

**Done when** the block lands within 2 mm of the mark 10 times in a row.

---

## Week 2: four arms, one plate

**Goal:** four arms work around one plate without colliding.

**Build**
- Three more arms.
- Second PCA9685 with the A0 pad bridged (address 0x41) for arm 4.
- The base board, with the arms on a 100 mm circle and the fixture in the centre.

**The anti-collision rule:** the centre of the plate is a shared zone, a 45 mm cylinder that holds at most 2 arms. Each arm owns its corner. The planner rejects any recipe that breaks the rule:

```
$ forge plan recipes/week2_zone_conflict.yaml
PLAN REJECTED: zone conflict at t=0.00s: arm1, arm2, arm3 are all in the centre zone (capacity 2).
Put these steps in sequence instead of in parallel.
```

**Simulation:** all four arms calibrate at once, then pairs visit the centre, then a hand-off and a press. The zone counter never goes above 2/2.

**Done when** `week2_four_arms.yaml` runs 5 times with no contact.

---

## Week 3: first product (LED keychain)

**Goal:** a part arrives, the arms assemble it, check it and unload it.

**Build**
- Conveyor: N20 motor, DRV8833, belt and two rollers.
- IR break-beam at the end of the belt (GPIO 26).
- Overhead USB camera, with ArUco markers 0–3 (4x4_50 dictionary) on the plate corners.
- Vacuum pump on arm 4 (MOSFET on GPIO 25), plus an LED tray and a cap tray.

**The recipe** (`recipes/led_keychain.yaml`, 12 steps, about 30 s)

| # | Arm | Action |
|---|---|---|
| 1 | cell | conveyor brings the housing until the IR sensor sees it |
| 2–3 | arm1 Loader | pick the housing, place it in the fixture |
| 4 | arm2 + arm4 | in parallel: arm2 clamps the housing, arm4 vacuums up an LED |
| 5 | arm4 | LED into the slot |
| 6–7 | arm1 | fetch the cap, set it on top |
| 8 | arm3 Tool | press the cap home (3 mm) |
| 9 | arm2 | release |
| 10 | arm4 | camera check: LED seated |
| 11–12 | arm4 | unload to the output tray |

Fixtures with funnels and V-guides matter more than servo precision. Hobby servos are only accurate to ±1–2 mm, so let the fixture finish the alignment.

**Done when** 10 keychains in a row pass the inspection step.

---

## Week 4: Claude takes the controls

**Goal:** you describe what you want, and Claude writes, checks and runs the recipe.

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

**Tools Claude gets:** `describe_cell`, `check_reach`, `list_recipes`, `get_recipe`, `plan_recipe`, `save_recipe`, `run_recipe` (needs `confirm=true`), `machine_status`, `stop`.

**What a session looks like.** Claude's first draft (`recipes/ai_first_draft.yaml`) asks arm 3 to unload. The planner answers:

```
step 12 (...and dropped in the output tray): arm3 cannot reach (105, 0, 35): wrist needs 191 mm, max is 152 mm
```

Claude switches that step to arm 4, which sits next to the tray. The plan is clean, Claude asks you, and the run starts.

**Safety layers, top to bottom**
1. The model can only submit recipes. It never sends raw servo commands.
2. The planner checks reach, joint limits and zone capacity for every 20 mm waypoint.
3. `run_recipe` refuses to run without explicit confirmation.
4. The ESP32 watchdog freezes the arms if the Pi stops talking for 1.5 s.
5. The hardware e-stop cuts servo power, and software cannot override it.

---

## After the prototype

- Swap the MG996R/MG90S servos for steppers or smart servos with position feedback (STS3215 or Dynamixel).
- Add tool changers (magnet + peg) so any arm can pick up a soldering pen, screwdriver or glue pen.
- Move motion planning to ROS 2 + MoveIt 2 for real collision meshes instead of zones.
- Scale the links: everything in `cell.yaml` is a parameter, and `forge budget` tells you which servos you need.
