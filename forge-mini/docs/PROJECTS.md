# Projects and scheduling

## Keeping every arm busy

Recipes are written as a plain list of steps. With `schedule: auto` (the default), the planner starts each step as early as it safely can, so arms work side by side instead of waiting in line. A step waits only for:

1. **Its own arm**, which must have finished its previous step.
2. **Parts, tools and bits it needs**, for example a pick of `m1` waits until `m1` exists, and `tool_change` to `pen` waits until the pen is back in the rack.
3. **Work on the same spot.** Steps within 20 mm of each other on the same part keep their recipe order, so screws go in after the board they hold and props after their motors. Work on different corners overlaps.
4. **Whole-part steps.** Placing a base part, `hold`, `release`, `inspect` and lifting the part out wait for everything before them on that part.
5. **Shared zones.** Each rack and tray admits one arm and the fixture three. The step is delayed until there is room.
6. **Clearance.** Every pair of tools (each a vertical segment from tip to coupler face) stays 20 mm apart, checked every 50 ms of the plan. A step that would come closer is pushed later.

If a step can never fit, for example a clamp parked where the iron must work, the planner says which arms collide and when, so you can move a hold point instead of guessing.

`schedule: sequential` runs steps strictly in order, which is useful for comparing:

| Job | One at a time | Auto | Saved |
|---|---|---|---|
| Week 2: four arms, two racks | 47.5 s | 35.6 s | 25% |
| Doweled bracket | 103.8 s | 84.3 s | 19% |
| Micro-drone | 152.3 s | 76.3 s | 50% |
| Desk robot head | 119.9 s | 71.7 s | 40% |
| LED bar | 120.7 s | 78.7 s | 35% |

`forge plan <recipe>` prints when each step starts, which arm runs it, how busy each arm is, and the sequential time for comparison. In the simulator, the **All arms at once / One at a time** switch plays both versions, and the per-arm chips show which arms are working, waiting or idle.

The bracket saves the least because almost every step works on the same two holes, so the steps really must happen in order. The drone saves the most because its four corners are independent.

## Micro-drone (`recipes/micro_drone.yaml`)

A 65 mm brushed quad, like the 3D-printed Arduino drone, scaled to the fixture.

- **You load:** printed X-frame (kit tray), flight-controller board (side pocket), 3 motors + 3 props (cups A and B at the back), 1 motor + 1 prop (cup C at the front).
- **Arm 3:** frame to fixture, board onto its standoffs, motor 4, prop 4.
- **Arms 1 and 2:** the other three motors and props, at the same time on opposite corners.
- **Arm 4:** hex bit drives two M2 board screws, then the iron solders the motor leads.
- One lead per motor is soldered to keep the recipe short; add four more pad locations for a real board.

## Desk robot head (`recipes/desk_bot_head.yaml`)

The head of the Arduino desk robot with an OLED face, assembled face-up.

- **You load:** printed head shell (kit tray), 0.96" OLED module (side pocket), two M2 screws (cup B).
- **Arm 3:** shell to fixture, OLED into the face recess.
- **Arm 2:** glue bead along two edges of the OLED.
- **Arm 1:** sets both screws with tweezers, later unloads with the gripper.
- **Arm 4:** drives the screws, then solders VCC, GND, SCL and SDA.
- The full robot body has faces that point sideways. That needs a wrist-roll joint, the first upgrade after the prototype.

## LED bar, soldered underneath (`recipes/led_bar.yaml`)

Through-hole LEDs, the way you would do them on a helping-hands station: insert from the top, flip, solder underneath.

- **You load:** 64 × 16 mm PCB (kit tray), two LEDs in cup A, two in the side pocket.
- **Arm 2:** the clamp holds the left end of the board for the whole job.
- **Arms 1 and 3:** place LEDs from both sides (tweezers and vacuum).
- **Arm 2** releases, then the **flip fixture** turns the board over (every arm must be out of the centre zone while it turns).
- **Arm 4:** the iron solders all 8 legs on the underside, then the board turns back and arm 1 unloads it.

## Writing your own project

1. Add the part positions to `locations:` in `cell.yaml` under a new prefix (for example `clock.`). The prefix is the "part" the scheduler reasons about.
2. Keep hold points at least 20 mm from every spot other arms work on.
3. Write the steps in a sensible order. Don't add `parallel` blocks; the scheduler finds the overlap.
4. Run `forge plan`. If it reports a collision or an unreachable point, move that location and try again.
5. Run `python sim/build.py` after adding the recipe to `JOBS` in that file to see it in 3D.
