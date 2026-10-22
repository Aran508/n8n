# Arm design

All four arms are identical: a base yaw joint, three pitch joints (shoulder, elbow, wrist) and a tool servo.
While working, the tool always points straight down (pitch −90°).

## Geometry

```
            elbow (MG90S)
               o
       L1 80  / \  L2 80
             /   \
 shoulder   o     o  wrist (SG90)
 (MG996R)   |     |
     h0 55  |     |  L3 45  -> tool tip
            |     v
 ===========#==========  work plate, z = 0
        base yaw (MG996R)
```

| Symbol | Length | Between |
|---|---|---|
| h0 | 55 mm | plate top and shoulder axis |
| L1 | 80 mm | shoulder and elbow |
| L2 | 80 mm | elbow and wrist |
| L3 | 45 mm | wrist and tool tip |

The four bases sit on a 100 mm circle around the plate centre, one per corner of a 300 × 300 mm board:

| Arm | Role | Base (x, y) mm | Faces | Tool | PCA9685 channels |
|---|---|---|---|---|---|
| arm1 | Loader | (−70.7, 70.7) | −45° | gripper | 0x40 ch 0–4 |
| arm2 | Holder | (70.7, 70.7) | −135° | gripper | 0x40 ch 5–9 |
| arm3 | Tool | (−70.7, −70.7) | 45° | press pad | 0x40 ch 10–14 |
| arm4 | Inspect | (70.7, −70.7) | 135° | vacuum + camera | 0x41 ch 0–4 |

## Joint conventions and limits

| Joint | Meaning | Joint range | Servo degrees |
|---|---|---|---|
| base | 0 = facing plate centre, + = counter-clockwise | −90…90 | 90 + angle |
| shoulder | upper arm angle above horizontal | 0…180 | angle |
| elbow | forearm relative to upper arm, negative = elbow-up | −180…0 | −angle |
| wrist | tool relative to forearm | −90…90 | 90 + angle |
| tool | gripper: 20 open, 95 closed | 0…180 | angle |

Add each arm's calibration error to `trims` in `controller/cell.yaml`.

## Inverse kinematics

Given a target (x, y, z) and tool pitch φ = −90°:

1. Move the target into the arm's frame. `base = atan2(fy, fx)`, `r = hypot(fx, fy)`.
2. Wrist point: `rw = r − L3·cos φ`, `zw = z − L3·sin φ − h0`.
3. Two-link solution: `cos θ2 = (rw² + zw² − L1² − L2²) / (2·L1·L2)`, and `θ2 = −acos(...)` for the elbow-up pose.
4. `θ1 = atan2(zw, rw) + atan2(L2·sin(−θ2), L1 + L2·cos θ2)`
5. `θ3 = φ − θ1 − θ2`

Targets are rejected when the wrist distance exceeds 95% of L1 + L2 (152 mm, away from the straight-arm singularity) or when any joint leaves its range. The rejection message says why, for example `arm3 cannot reach (105, 0, 35): wrist needs 191 mm, max is 152 mm`.

Code: `controller/forge/kinematics.py`. Try it with `forge ik arm1 0 0 6`.

## Reach

| | |
|---|---|
| Fully stretched, horizontal | 205 mm |
| Max wrist distance used | 152 mm |
| Arm base to plate centre | 100 mm |
| Shared centre zone | 45 mm radius, at most 2 arms |

Each arm reaches the whole fixture, its own corner and the trays next to it. It cannot reach the opposite corner, and the planner enforces this.

## Torque check (`forge budget`)

Worst case is the arm stretched out flat holding a 30 g part. Masses: MG90S 13.4 g, SG90 9 g, printed links 8 / 7 / 6 g.

| Joint | Servo | Needs | Stall @ 6 V | Load |
|---|---|---|---|---|
| shoulder | MG996R | 1.26 kg·cm | 11.0 kg·cm | 11% |
| elbow | MG90S | 0.63 kg·cm | 2.2 kg·cm | 29% |
| wrist | SG90 | 0.17 kg·cm | 1.8 kg·cm | 9% |

The rule is to stay under 50% of stall. An MG90S on the shoulder would be at 57%, so the shoulder (and the base, which carries the whole arm) use MG996Rs.

## Bill of materials (whole cell)

| Item | Qty | ~USD | ~INR |
|---|---|---|---|
| MG996R servo (base + shoulder) | 8 | $40 | ₹2,800 |
| MG90S servo (elbow) | 4 | $12 | ₹800 |
| SG90 servo (wrist + tool) | 8 | $16 | ₹1,000 |
| PCA9685 16-channel driver | 2 | $8 | ₹600 |
| ESP32 dev board | 1 | $6 | ₹450 |
| Raspberry Pi 5 (4 GB), or a Pi 4 | 1 | $60 | ₹5,500 |
| USB camera, 1080p | 1 | $15 | ₹1,200 |
| 6 V 10 A supply, e-stop button, wiring | 1 | $22 | ₹1,700 |
| Conveyor: N20 motor, DRV8833, belt, rollers | 1 | $15 | ₹1,000 |
| IR break-beam sensor | 1 | $3 | ₹200 |
| Mini vacuum pump + MOSFET | 1 | $8 | ₹600 |
| Filament, M3 screws, bearings, base board | – | $25 | ₹2,000 |
| **Total** | | **~$230** | **~₹18,850** |

Power note: an MG996R draws about 1 A while moving and up to 2.5 A when stalled. Keep servo power on its own supply, joined to the ESP32 only at ground. Put the e-stop in series with the servo supply so that pressing it cuts power in hardware.
