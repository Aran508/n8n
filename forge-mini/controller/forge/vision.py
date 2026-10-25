"""Overhead camera: ArUco markers give plate coordinates; checks are simple and explainable.

Markers 0-3 sit on the four plate corners at known world positions, so a
homography maps camera pixels to millimetres on the plate. Requires
opencv-contrib-python (cv2.aruco).
"""

from __future__ import annotations

import numpy as np

PLATE_MARKERS_MM = {0: (-130, 130), 1: (130, 130), 2: (130, -130), 3: (-130, -130)}


class Camera:
    def __init__(self, index: int = 0):
        import cv2

        self.cv2 = cv2
        self.cap = cv2.VideoCapture(index)
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        self.H = None

    def frame(self):
        ok, img = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        return img

    def calibrate(self) -> None:
        """Fit pixel -> plate-mm homography from the four corner markers."""
        corners, ids, _ = self.detector.detectMarkers(self.frame())
        if ids is None:
            raise RuntimeError("no markers visible")
        src, dst = [], []
        for c, i in zip(corners, ids.flatten()):
            if int(i) in PLATE_MARKERS_MM:
                src.append(c[0].mean(axis=0))
                dst.append(PLATE_MARKERS_MM[int(i)])
        if len(src) < 4:
            raise RuntimeError(f"need 4 corner markers, saw {len(src)}")
        self.H, _ = self.cv2.findHomography(np.array(src), np.array(dst))

    def to_plate(self, px: float, py: float) -> tuple[float, float]:
        if self.H is None:
            self.calibrate()
        p = self.H @ np.array([px, py, 1.0])
        return float(p[0] / p[2]), float(p[1] / p[2])

    def find_marker(self, marker_id: int) -> tuple[float, float] | None:
        """Parts carry small markers (ids >= 10); returns their plate XY in mm."""
        corners, ids, _ = self.detector.detectMarkers(self.frame())
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if int(i) == marker_id:
                return self.to_plate(*c[0].mean(axis=0))
        return None

    def bright_spot(self, at_mm: tuple[float, float], radius_mm: float = 6) -> bool:
        """True if there is a bright LED-coloured blob near `at_mm` (used for led_seated)."""
        img = self.frame()
        hsv = self.cv2.cvtColor(img, self.cv2.COLOR_BGR2HSV)
        mask = self.cv2.inRange(hsv, (0, 0, 220), (180, 80, 255))
        ys, xs = np.nonzero(mask)
        for x, y in zip(xs[::25], ys[::25]):
            px, py = self.to_plate(x, y)
            if np.hypot(px - at_mm[0], py - at_mm[1]) < radius_mm:
                return True
        return False


def make_inspector(camera: Camera):
    def inspect(data: dict) -> bool:
        x, y, _ = data["at"]
        if data.get("check") == "led_seated":
            return camera.bright_spot((x, y))
        return True

    return inspect
