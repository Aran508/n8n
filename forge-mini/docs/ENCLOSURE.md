# The miniature robot cell (enclosure and safety)

Everything an industrial multi-arm robot cell has, built at desk size around the 300 × 300 mm base.

```
                 overhead LED bar + camera          tower light (R/Y/G)
        ┌──────────────────────────────────────────────●──┐
        │  fume fan + carbon filter                        │
        │                                                  │   ┌──────────┐
        │        arm1 ●───────────────● arm2               │   │ 7" HMI   │
        │  rack L        [flip fixture]        rack R      │   │ START    │
        │        arm3 ●───────────────● arm4               │   │ STOP     │
        │                                                  │   │ E-STOP   │
        └─────────┤▒▒ loading window ▒▒├─────── door ──────┘   └──────────┘
                    light curtain        reed-switch interlock
                    kit drawer + ID tag
   ▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚ yellow/black floor marking ▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚▚
```

Outside size: 410 × 410 × 350 mm.

## Parts

| Industrial cell | Mini version | ~INR |
|---|---|---|
| Safety enclosure | 2020 aluminium extrusion (12 × 350–410 mm), corner brackets, 3 mm clear acrylic panels | ₹3,500 |
| Safety door with interlock | Hinged acrylic door, magnetic reed switch at the latch | ₹150 |
| Safety light curtain | 6 IR break-beam pairs (3 mm) across the 100 mm loading window | ₹600 |
| Material loader | Sliding kit drawer on two rails, with a printed QR / ArUco tag (or an RC522 RFID reader + cards, ₹250) | ₹300 |
| Status tower light | 3-colour 12 V LED stack, or three 10 mm LEDs in a printed tube | ₹350 |
| Overhead light and camera | 12 V LED bar + the 1080p USB camera on the roof rail | ₹400 |
| Control panel and HMI | Raspberry Pi 7" touchscreen, START / STOP buttons, latching E-STOP | ₹4,500 |
| Air filtration | 80 mm 12 V fan + carbon filter pad on the back wall | ₹500 |
| Floor marking | Yellow/black hazard tape around the footprint | ₹100 |
| 4 × logic-level MOSFET boards | Switch tower, fan and light from the ESP32 | ₹250 |
| **Total** | | **~₹10,650 (~$128)** |

## Wiring (ESP32)

| Signal | Pin | Wiring | Meaning |
|---|---|---|---|
| E-STOP | GPIO 27 | NC contact to GND; the second NC contact is in series with the servo supply | latched stop, servo power cut in hardware |
| Door | GPIO 13 | reed switch to GND, internal pull-up | LOW = door shut. Unplugged reads open, which is the safe default |
| Light curtain | GPIO 14 | receivers' outputs in series to GND, internal pull-up | LOW = all beams clear |
| START / STOP | GPIO 16 / 17 | buttons to GND | START emits an event; STOP freezes arms and tools |
| Tower red / yellow / green | GPIO 18 / 19 / 23 | MOSFET boards | see below |
| Fume fan | GPIO 4 | MOSFET board | on while a recipe uses the iron |
| Overhead light | GPIO 15 | MOSFET board | on while running |

## Safety behaviour

| Event | What happens | How to recover |
|---|---|---|
| Door opened while anything moves or spins | arms freeze, tool signals off (spindle, pump, iron heater, feeder), tower red, `interlock` event, the run aborts | close the door, then `{"cmd":"reset"}` |
| Hand in the loading window while running | same as the door | clear the window, reset |
| STOP button | arms freeze, tools off, tower yellow, run aborts | start the recipe again |
| E-STOP | servo power cut in hardware, PWM off, tower red | release the button, reset |
| Pi stops talking for 1.5 s | watchdog freezes arms and turns tools off | restart the run |
| Starting with the door open or the window blocked | `forge run` refuses to start | close the door / clear the window |

Tower light: **green** = running, **yellow** = waiting for you (part finished, or stopped), **green blinking** = idle and ready, **red** = e-stop or interlock.

Loading parts is safe: the curtain only trips while something is moving, so you can fill the kit drawer between jobs.

## Try it in the simulator

Under the 3D view, **Open the door** and **Reach into the loading window** freeze the job, show the stop reason and turn the tower red. **Reset** continues. **Show enclosure** hides the walls. Posts on the side facing the camera hide themselves, so you can always see in.
