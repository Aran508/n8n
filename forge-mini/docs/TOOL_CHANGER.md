# Tool changer, tool racks and bit block

Every arm ends in the same coupler, and every tool carries the matching plate. Any arm can pick up any tool from a rack it can reach, and the spindle picks cutters from a bit block, much like a reamer set.

## Coupler (one per arm, on the wrist)

| Part | What it does |
|---|---|
| 3 × 6 mm steel balls, arm side | Sit in 3 V-grooves on the tool plate: a kinematic mount that seats in the same place every time (about ±0.05 mm) |
| 2 × 10×3 mm neodymium magnets | Pull the plates flat before the latch closes |
| SG90 cam latch (joint 5) | 20° = open, 105° = locked; its hook catches a lip on the tool plate |
| 4 pogo pins | 6 V, GND, SIG, ID |

Wrist axis to coupler face: **22 mm**. Coupler mass on the arm: about 10 g plus the latch servo.

### Pogo pins

- **SIG** is a servo-style PWM line from the PCA9685 (channel 6 of each arm). A gripper uses it directly. The spindle, vacuum pump and glue plunger each have a tiny brushed ESC that reads it as a speed.
- **ID**: each tool has one resistor from ID to GND. The ESP32 has a 10 kΩ pull-up to 3.3 V on each arm's ID line (GPIO 32–35), so it can tell which tool is mounted:

| Tool | ID resistor |
|---|---|
| gripper (A and B) | 1 kΩ |
| vacuum | 2.2 kΩ |
| pen | 3.3 kΩ |
| probe | 4.7 kΩ in series with a normally-closed switch (a touch reads as open) |
| clamp | 6.8 kΩ |
| spindle | 10 kΩ |
| glue | 15 kΩ |
| nothing | open |

After every pick-up the executor reads the ID and stops the run if it doesn't match. After every drop-off it checks that the line reads open.

## Tool racks

Each rack is a shelf 40 mm above the plate with four holes. A tool hangs through its hole by its coupler plate, like a reamer in a stand. Neighbouring arms share a rack:

| Rack | Arms | Slots (x, y) | Tools |
|---|---|---|---|
| left (rack_L) | arm1, arm3 | (−104, ±30), (−104, ±10) | gripper_A, vacuum, pen, probe |
| right (rack_R) | arm2, arm4 | (104, ±30), (104, ±10) | gripper_B, clamp, spindle, glue |

The slot positions are not arbitrary. They are the area where both neighbouring arms can reach every slot with every tool. Their base servos must stay within ±90°, and their wrists must stay within ±90° while lifting a 48 mm tool out of its hole. The test `test_every_tool_slot_reachable_by_both_neighbours` checks this.

**Pick up a tool:** move above the slot bare (latch open) → lower onto the plate → close the latch → read the ID → lift until the tool clears the shelf → park.
**Return a tool:** lift → move above the slot → lower until the plate sits on the shelf → open the latch → lift away bare.

Each rack is a shared zone with capacity 1, so two arms never reach into the same rack at once.

## Bit block (arm2, arm4)

A 16 mm wide block beside the right rack. Bits stand shank-up in holes, with a stripper bar just above their collars.

| Bit | Use | Stick-out |
|---|---|---|
| ream_2_0 | finish 1.8 mm printed pilot holes to an exact 2.0 mm (dowel press fit) | 16 mm |
| ream_3_0 | finish 2.8 mm holes to 3.0 mm (M3 clearance, 3 mm pins) | 18 mm |
| drill_1_5 | new holes in plastic, wood or PCB | 14 mm |
| csink_90 | chamfer hole edges so pins start straight | 10 mm |
| hex_1_5 | drive M2 socket screws | 14 mm |

The spindle's collet is a magnet with a V-notch. **Pick:** lower the spindle onto the shank, and the magnet grabs it. **Return:** lower the bit into its hole, slide 12 mm sideways under the stripper bar (which pulls it off the magnet), then lift.

## Why reaming works on a small servo arm

Drilling solid material needs a lot of downward force. Reaming a printed pilot hole removes only 0.1 mm per side. `forge budget` shows the arm can push down with about **3.4 N** at the fixture before any joint passes 50% of stall. That is plenty for reaming plastic or wood, but not for metal. Keep the feed slow (2 mm/s in `cell.yaml`) and let the reamer follow the pilot hole.

## Adding a tool

1. Give it a coupler plate and an unused ID resistor value.
2. Add it to `tools:` in `cell.yaml` with its rack slot, length (coupler face to tip) and mass.
3. Run `forge budget` to check servo load and `pytest` to check the slot is reachable.
4. If it does something new, add an action for it in `forge/planner.py` and list its type in `NEEDS`.
