# Smartwatch: planned for the cell

The perfboard smartwatch from the video: an ESP32-C3 SuperMini, a USB-C TP4056 charger module, a 0.96" OLED stacked on top, three 12 mm buttons, a P-MOSFET power switch with a 10 kΩ pull-up, a small LiPo, a perfboard back cover and a velcro strap.

**Verdict: yes, the cell can build it.** Two short jobs stay with a person (the battery plug and the strap), and the board needs a few small design changes so every step can be done from above or after a flip. Reliable 2.54 mm-pitch joints need the smart-servo + camera precision upgrade (see `CAPABILITIES.md`). The hobby-servo prototype can rehearse the whole sequence on a dummy board.

## Step by step: the video vs the cell

| In the video (by hand) | In the cell | How |
|---|---|---|
| Solder header pins onto the ESP32-C3 | avoid | buy the module with headers already soldered |
| Place ESP32, buttons, resistor, MOSFET | **cell** | vacuum (ESP32), gripper (buttons), tweezers (small parts), pressed into the holes |
| Solder every leg underneath | **cell** | flip fixture, then **two irons at once**: 39 legs |
| Wire the charger module | **cell** | castellated USB-C charger soldered on its 4 pads from the top |
| Point-to-point wires underneath | **cell, redesigned** | 7 drag-soldered tracks along the pad rows (or a small custom PCB, below) |
| Stack the OLED on top | **cell, redesigned** | a 4-pin socket on the board; the OLED presses in from above |
| Flash the firmware over USB-C | **cell** | USB-C programming dock on a servo slide; the Pi flashes and reads the self-test |
| Plug in and stick down the battery | **you, ~30 s** | a flexible lead is hard for these arms; the arms freeze, the tower goes yellow, press START when done |
| Screw on the back cover | **cell** | flip, back cover on pre-fitted standoffs, 2 M2 screws driven from below |
| Cut and fit the velcro strap | **you** | soft material, threaded by hand |

## Design changes for the cell

1. **Pre-soldered headers** on the ESP32-C3 SuperMini.
2. **Vertical sockets** for the OLED (4-pin female header) and the battery (vertical JST-PH), so both connect by pushing straight down.
3. **Tracks instead of wires.** The hand-routed wires become drag-soldered tracks along perfboard pad rows. The tracks in `make_smartwatch.py` are representative; generate yours from your schematic's netlist.
4. **Better: a small custom PCB.** A 40 × 40 mm two-layer board costs about ₹200 for five. It removes the tracks entirely (43 joints, no drag soldering), and its silkscreen and fiducials help the camera.
5. **Castellated charger module**, soldered on its pads from the top.
6. **Back cover with standoffs already fitted**, so it drops straight on.

## Additions to the cell

| Addition | Why | ~INR |
|---|---|---|
| Second soldering iron (`iron_B`) on a one-slot stand at the back (arms 1 and 2) | two irons solder the underside in parallel | ₹2,500 |
| USB-C programming dock: a USB-C plug on a printed slide driven by an SG90 (board0 ch 13) | flashing and self-test inside the cell | ₹300 |
| Flip fixture servo (board0 ch 12) and a 40 × 40 mm cradle insert | holds the watch board during flips | ₹150 |

## The plan (`recipes/smartwatch.yaml`)

`python recipes/make_smartwatch.py` generates the recipe: 66 steps, 43 solder joints, 7 tracks and 4 flips.

| Time (auto schedule) | What happens |
|---|---|
| 0–40 s | Tool up. Board into the flip fixture, then ESP32, three buttons, resistor, MOSFET, OLED socket and JST socket placed; charger module placed |
| ~51 s | Iron solders the charger's 4 pads on top |
| ~69 s | **Flip**: legs point up |
| 70–240 s | **Two irons at once**: arm 4 does the ESP32, MOSFET and button 3; arm 2 does buttons 1–2, resistor, JST and OLED socket; then 7 tracks |
| ~245 s | Flip back. OLED pressed into its socket |
| ~251 s | **USB-C dock** slides in: flash firmware and self-test; camera checks the watch face |
| ~272 s | **You**: plug in the battery and lay it on the back; press START. Nothing moves, not even the fixture, until you do |
| ~302 s | Flip, back cover on, 2 M2 screws driven from below, flip back |
| ~342 s | Finished watch to the tray; you fit the strap |

**About 5¾ minutes per watch** (including your 30 s), against 7.9 minutes if the arms took turns. Arms 2 and 4 are each busy about 45% of the time; soldering is the bottleneck.

## Firmware and self-test

`firmware/watch_example/watch_example.ino` is a minimal watch: a clock on the OLED, with buttons to set it. At boot it prints `SELFTEST OK`, or `SELFTEST FAIL` and the reason: the OLED is missing on I²C, or a button reads stuck low (usually a solder bridge). `forge.programmer` flashes it with esptool through the dock and waits for that line:

```
forge run recipes/smartwatch.yaml --port /dev/ttyUSB0 --camera 0 \
    --flash-port /dev/ttyACM0 --firmware-dir ../firmware/watch_example/build
```

## Parts per watch (~₹970)

ESP32-C3 SuperMini with headers ₹350 · USB-C TP4056 charger ₹40 · 0.96" I²C OLED ₹180 · 3 × 12 mm tactile buttons ₹15 · P-MOSFET (AO3401) on a 3-pin breakout ₹30 · 10 kΩ resistor ₹1 · 4-pin socket + vertical JST-PH ₹30 · 300 mAh LiPo with JST ₹200 · 2 × 40 × 40 mm perfboards ₹40 · M2 screws and standoffs ₹30 · velcro strap ₹50.
