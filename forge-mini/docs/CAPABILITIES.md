# What Forge Mini can make

## Short answer

| Question | Answer |
|---|---|
| Small (minute) assembly? | **Modules, motors, screws and printed parts: yes.** Through-hole parts and small SMD need the precision upgrade below. Fine-pitch chips: no. |
| Soldering? | **Yes**, with the iron tool: wires to pads and through-hole legs. Reliable joints on 1.5 mm pads need the precision upgrade. Small SMD parts are better placed into solder paste and reflowed on a hot plate. |
| Work underneath the product? | **Yes**, with the flip fixture: the part turns over, the arms work on the underside, and it turns back. See the LED bar project. |
| A complete product? | **Yes, if** every step can be done from above or after a flip and the parts come as a kit (printed parts, ready-made modules, screws, through-hole parts, wires). |

## Accuracy, computed (`forge precision`)

The planner knows the exact arm geometry, so the tool-tip error at the fixture can be computed from each servo's angle error. "Typical" adds the four joint errors as root-sum-square; "worst" assumes they all point the same way.

| Level | Typical | Worst |
|---|---|---|
| Hobby servos, open loop (this prototype) | 2.89 mm | 5.61 mm |
| Hobby servos + camera correction | 0.90 mm | 1.75 mm |
| Smart servos (STS3215), open loop | 0.72 mm | 1.40 mm |
| Smart servos + camera correction | 0.15 mm | 0.28 mm |

| Job | Needs | Cheapest level that does it |
|---|---|---|
| Modules, motors, M2 screws (with lead-in chamfers) | ±1.5 mm | hobby servos + camera correction |
| Through-hole LEDs / headers into chamfered holes | ±0.5 mm | smart servos + camera correction |
| Iron on a 1.5 mm solder pad | ±0.5 mm | smart servos + camera correction |
| SMD 0805 / 0603 into solder paste, then hot-plate reflow | ±0.2 mm | smart servos + camera correction |
| Fine-pitch IC (0.5 mm pitch) | ±0.05 mm | not with arms like these; order the board assembled |

Why hobby servos are limited: they have about 5 µs of deadband (0.45°) plus gear backlash. At 150 mm from the shoulder, 0.8° is about 2 mm at the tool tip. A camera can measure and correct the error, but a hobby servo still moves in deadband-sized steps.

Mechanical tricks that work at every level:
- **Lead-in chamfers and funnels** on every hole and pocket, so a part slides home even when the arm is a little off. The reamer and countersink in the bit block exist for this.
- **Compliant tools**: a spring in the tweezers, gripper and press pad lets the part settle.
- **Fixture pockets** that locate the part, so the arms never need to measure it.

## The precision upgrade (v2 arms)

| Change | Why | Cost |
|---|---|---|
| Feetech STS3215 smart servos on shoulder, elbow, wrist and base (16 total) | 12-bit magnetic encoder, position read-back, much less backlash | ~$240 / ₹20,000 |
| Bus servo adapter (half-duplex UART) on ESP32 Serial2 | STS servos use a serial bus instead of PWM | ~$8 / ₹700 |
| Camera look-then-correct | Before the final 5 mm, the overhead camera finds the part's ArUco marker and the tool's marker and nudges the target | software |
| Hot plate (100 × 100 mm) and a solder paste stencil | SMD boards: the arm places parts into paste; reflow pulls them into place | ~$25 / ₹2,000 |

The planner, recipes and simulator stay the same. Only `forge budget` and `forge precision` need the new servo data, and the firmware's servo output changes from PCA9685 PWM to the Feetech SCServo library. That firmware variant is not written yet.

## Underside work: the flip fixture

The arms only reach down, so the centre fixture is a small trunnion: a cradle on two bearings turned by one MG996R (or a stepper). A recipe step `flip: {at: led}` turns the part 180° in 1.6 s.

The planner makes it safe:
- Every clamp must be released first ("arm2 is still holding the part; add release before flipping").
- No arm may be inside the centre zone while the cradle turns, and the scheduler waits until they have left.
- It remembers the part is upside down and refuses to unload it until it is turned back.
- A point (x, y, z) on the part ends up at (x, −y, 2·axis_z − z), so underside locations are easy to write. See `led.bot.*` in `cell.yaml`.

## What a "complete product" means here

The cell assembles **kits**. You supply:
- **3D-printed parts**, ideally with pilot holes, chamfers and snap features designed for the cell.
- **Ready-made modules**: Arduino Nano, OLED, Bluetooth module, motor driver. The cell does not make PCBs.
- **Fasteners and small parts**: M2 screws, dowel pins, LEDs, motors, props.
- **Pre-cut, pre-tinned wires** for solder joints.

Still a job for a person (or a much bigger machine): routing loose wires through tight channels, cables with crimped connectors, fine-pitch soldering, and anything that has to be screwed in from the side. Sideways work needs a wrist-roll joint, the next upgrade after the smart servos.
