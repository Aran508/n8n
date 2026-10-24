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

## Gestures

| Gesture | Action |
| --- | --- |
| 🙌 Two hands | Pull apart to explode the cylinder, push together to reassemble. A fast pull sets off a spark blast. |
| ✋ Open palm | Move your hand to rotate the model |
| ✊ Fist | Hold to clamp the parts back together |
| 🤏 Pinch | Pinch and move up/down to zoom |

No camera? Drag to rotate, scroll to zoom, use the slider, press `E` to explode,
and `H` to switch between hologram and realistic metal view.
