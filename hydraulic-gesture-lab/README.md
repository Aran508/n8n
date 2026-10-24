# Hydraulic Gesture Lab

A holographic, Stark-workshop-style 3D tie-rod hydraulic cylinder that you take
apart with your hands through the webcam.

## Run it

Browsers only allow the camera on `localhost` or `https`, so serve the folder:

```bash
cd hydraulic-gesture-lab
python3 -m http.server 8000
# open http://localhost:8000 and press "Start camera"
```

Hand tracking (MediaPipe Hand Landmarker) runs fully in the browser; no video is uploaded.

## Gestures (one hand)

| Gesture | Action |
| --- | --- |
| ☝️ Point | Aim the cursor at a part to highlight it |
| 🤏 Pinch | Select the part you're aiming at; keep pinching and move to pull it out. Pinch empty space to deselect |
| ✋ Open palm | Move to turn the model; move your hand closer to the camera to zoom in |
| ✌️ Peace sign | Hold to explode the cylinder (a fast explode sets off a spark blast) |
| ✊ Fist | Hold to reassemble everything |

No camera? Click a part to select it, drag a part to pull it out, drag empty space
to rotate, scroll to zoom. `E` explodes, `R` resets, `Esc` deselects, `H` switches
between hologram and realistic metal view.
