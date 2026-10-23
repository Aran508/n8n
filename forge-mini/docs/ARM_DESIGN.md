# Arm design

All four arms are identical: base yaw, shoulder, elbow and wrist, then a tool-changer coupler with its latch servo. While working, the tool always points straight down (pitch −90°).

## Geometry

```
            elbow (MG996R)
               o
       L1 80  / \  L2 80
             /   \
 shoulder   o     o  wrist (MG90S)
 (MG996R)   |     |  coupler 22 (latch SG90)
     h0 55  |     =  <- coupler face
            |     |  tool 30-48 (+ bit 10-18 on the spindle)
 ===========#=====v====  work plate, z = 0
        base yaw (MG996R)
```

| Symbol | Length | Between |
|---|---|---|
| h0 | 55 mm | plate top and shoulder axis |
| L1 | 80 mm | shoulder and elbow |
| L2 | 80 mm | elbow and wrist |
| coupler | 22 mm | wrist axis and coupler face |
| tool | 30–48 mm | coupler face and tool tip (see `forge tools`) |

The last link is **coupler + tool + bit**, and it changes with every tool change. The planner always solves the kinematics with whatever is mounted at that moment.

## Layout (300 × 300 mm base)

```
              back: pin cup · finished tray
     arm1 Assembler ●───────────● arm2 Holder
 left rack  |            [ fixture ]            |  right rack + bit block
     arm3 Loader    ●───────────● arm4 Machinist
              front: kit tray (you load parts here)
```

| Arm | Role | Base (x, y) mm | Faces | Rack | PCA9685 channels | ID pin |
|---|---|---|---|---|---|---|
| arm1 | Assembler | (−70.7, 70.7) | −45° | left | 0x40 ch 0–5 | GPIO 32 |
| arm2 | Holder | (70.7, 70.7) | −135° | right | 0x40 ch 6–11 | GPIO 33 |
| arm3 | Loader | (−70.7, −70.7) | 45° | left | 0x41 ch 0–5 | GPIO 34 |
| arm4 | Machinist | (70.7, −70.7) | 135° | right + bit block | 0x41 ch 6–11 | GPIO 35 |

The channels for each arm, in order, are base, shoulder, elbow, wrist, latch and tool signal.

## Joint conventions and limits

| Joint | Meaning | Joint range | Servo degrees |
|---|---|---|---|
| base | 0 = facing plate centre, + = counter-clockwise | −90…90 | 90 + angle |
| shoulder | upper arm angle above horizontal | 0…180 | angle |
| elbow | forearm relative to upper arm, negative = elbow-up | −180…0 | −angle |
| wrist | tool relative to forearm | −90…90 | 90 + angle |
| lock | coupler latch: 20 open, 105 locked | 0…180 | angle |

Add each arm's calibration error to `trims` in `controller/cell.yaml`.

## Inverse kinematics

Given a tool-tip target (x, y, z), tool pitch φ = −90° and last-link length l3 = coupler + tool + bit:

1. Move the target into the arm's frame. `base = atan2(fy, fx)`, `r = hypot(fx, fy)`.
2. Wrist point: `rw = r − l3·cos φ`, `zw = z − l3·sin φ − h0`.
3. `cos θ2 = (rw² + zw² − L1² − L2²) / (2·L1·L2)`, and `θ2 = −acos(...)` for the elbow-up pose.
4. `θ1 = atan2(zw, rw) + atan2(L2·sin(−θ2), L1 + L2·cos θ2)`
5. `θ3 = φ − θ1 − θ2`

Targets are rejected when the wrist distance exceeds 95% of L1 + L2 (152 mm) or any joint leaves its range, and the message says why. Try `forge ik arm4 -12 0 5 --tool spindle`.

## Servo budget (`forge budget`)

Worst case is the arm stretched flat with the coupler, a 22 g gripper and a 30 g part:

| Joint | Servo | Needs | Stall @ 6 V | Load |
|---|---|---|---|---|
| shoulder | MG996R | 2.20 kg·cm | 11.0 kg·cm | 20% |
| elbow | MG996R | 0.99 kg·cm | 11.0 kg·cm | 9% |
| wrist | MG90S | 0.29 kg·cm | 2.2 kg·cm | 13% |

No joint goes above 20% with any of the ten tools. The tool changer adds weight at the end of the arm, so the elbow is an MG996R. With an MG90S elbow it would sit at 44%, and the arm could push down with only 0.7 N. With the MG996R it can push **3.4 N** at the fixture, enough to ream plastic.

## Bill of materials (whole cell)

| Item | Qty | ~USD | ~INR |
|---|---|---|---|
| MG996R servo (base, shoulder, elbow) | 12 | $60 | ₹4,200 |
| MG90S servo (wrist) | 4 | $12 | ₹800 |
| SG90 servo (4 latches, 2 gripper jaws, tweezers, glue plunger) | 8 | $16 | ₹1,000 |
| PCA9685 16-channel driver | 2 | $8 | ₹600 |
| ESP32 dev board | 1 | $6 | ₹450 |
| Raspberry Pi 5 (4 GB), or a Pi 4 | 1 | $60 | ₹5,500 |
| USB camera, 1080p | 1 | $15 | ₹1,200 |
| 6 V 15 A supply, e-stop button, wiring | 1 | $28 | ₹2,200 |
| Tool changers: steel balls, magnets, pogo pins, ID resistors (4 arm sides, 10 tool plates) | – | $23 | ₹1,700 |
| Tool actuators: N20 1000 rpm motor, micro vacuum pump, 2 mini brushed ESCs | – | $16 | ₹1,300 |
| Bit set: 2.0 and 3.0 mm reamers, 1.5 mm drill, 90° countersink, 1.5 mm hex | – | $16 | ₹1,300 |
| Probe microswitch, fine-liner pen, 3 ml syringe, precision tweezers | – | $7 | ₹550 |
| USB-C mini soldering iron + N20 solder feeder + 0.5 mm solder | 1 | $30 | ₹2,500 |
| Filament, M3 screws, bearings, base board | – | $30 | ₹2,400 |
| **Total (arms and tools)** | | **~$327** | **~₹25,600** |
| Miniature robot cell (enclosure, interlocks, tower, HMI; see `ENCLOSURE.md`) | – | ~$128 | ~₹10,650 |
| Flip fixture: printed cradle, 2 × 608 bearings, 1 × MG996R | – | ~$8 | ~₹650 |
| **Total with cell** | | **~$463** | **~₹36,900** |

**Power:** 12 MG996Rs draw about 1 A each while moving, so use a 6 V supply rated for at least 15 A. Keep it separate from the ESP32 and Pi, joined only at ground. Put the e-stop in series with the servo supply so that pressing it cuts power in hardware.
