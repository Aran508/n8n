"""Plays a planned Timeline on the real machine over USB serial."""

from __future__ import annotations

import json
import queue
import threading
import time
from typing import Callable

from .config import Cell
from .planner import Timeline

ARM_INDEX = {"arm1": 1, "arm2": 2, "arm3": 3, "arm4": 4}


class MachineError(RuntimeError):
    pass


class SerialLink:
    """JSON-lines link to the ESP32. Replies and events are read on a thread."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial, only needed for real hardware

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.replies: queue.Queue = queue.Queue()
        self.events: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        self._alive = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._pinger, daemon=True).start()

    def _reader(self):
        while self._alive:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            (self.events if "event" in msg else self.replies).put(msg)

    def _pinger(self):  # keeps the firmware watchdog happy between moves
        while self._alive:
            time.sleep(0.5)
            try:
                self.send({"cmd": "ping"})
            except MachineError:
                pass

    def send(self, msg: dict, timeout: float = 1.0) -> dict:
        with self._lock:
            self.ser.write((json.dumps(msg) + "\n").encode())
            try:
                reply = self.replies.get(timeout=timeout)
            except queue.Empty:
                raise MachineError(f"no reply to {msg['cmd']}") from None
        if not reply.get("ok"):
            raise MachineError(f"{msg['cmd']} rejected: {reply.get('err')}")
        return reply

    def close(self):
        self._alive = False
        self.ser.close()


def play(
    cell: Cell,
    timeline: Timeline,
    link: SerialLink,
    inspector: Callable[[dict], bool] | None = None,
    log: Callable[[str], None] = print,
    flasher: Callable[[dict], bool] | None = None,
) -> None:
    """Send every segment and tool signal at its planned time.

    After each tool change the ESP32 reads the tool's ID resistor and the run
    stops if the wrong tool (or no tool) is on the coupler. Probe events check
    that the probe really touched, and inspect events call `inspector`
    (e.g. forge.vision); a failed check aborts the run.
    """
    status = link.send({"cmd": "status"})
    if status.get("door_open"):
        raise MachineError("the enclosure door is open; close it before starting")
    if status.get("curtain_broken"):
        raise MachineError("something is in the loading window (light curtain blocked)")
    if status.get("estop") or status.get("interlock"):
        raise MachineError("clear the e-stop / interlock and send reset first")
    solders = any(e.type == "solder" for e in timeline.events)
    link.send({"cmd": "light", "on": True})
    if solders:
        link.send({"cmd": "fan", "on": True})  # fume extractor while the iron is in use
    link.send({"cmd": "tower", "state": "run"})
    try:
        _play(cell, timeline, link, inspector, log, flasher)
    except MachineError:
        link.send({"cmd": "tower", "state": "fault"})
        raise
    finally:
        if solders:
            link.send({"cmd": "fan", "on": False})
    link.send({"cmd": "tower", "state": "wait"})  # yellow: part ready, waiting for the operator
    log("done: take the part from the finished tray, then press START or run the next job")


def _play(cell, timeline, link, inspector, log, flasher=None) -> None:
    items = [(s.t0, 0, "move", s) for s in timeline.segments]
    items += [(e.t, 1, "event", e) for e in timeline.events]
    items.sort(key=lambda i: (i[0], i[1]))

    touched: dict[str, float] = {}
    t_start = time.monotonic()
    offset = 0.0  # how far the real run has drifted from the plan (people, flashing)
    flips = 0
    for t, _, kind, obj in items:
        wait = t_start + t + offset - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        _check_events(link, touched)
        if kind == "move":
            link.send({"cmd": "move", "arm": ARM_INDEX[obj.arm], "servo": obj.servos, "ms": int(obj.dur * 1000)})
            continue
        e, d = obj, obj.data
        if e.type == "step":
            log(f"[{t:6.2f}s] step {d['index']}: {d['label']}")
        elif e.type == "signal":
            link.send({"cmd": "tool", "arm": ARM_INDEX[d["arm"]], "value": d["value"]})
        elif e.type == "id_check":
            time.sleep(0.15)  # let the latch settle before reading the ID resistor
            ohms = link.send({"cmd": "status"})["tool_ohms"][ARM_INDEX[d["arm"]] - 1]
            if ohms is None or abs(ohms - d["ohms"]) > 0.25 * d["ohms"]:
                seen = "nothing" if ohms is None else f"{ohms / 1000:.1f} kOhm"
                raise MachineError(f"{d['arm']} should now hold {d['tool']} ({d['ohms'] / 1000:.1f} kOhm) but reads {seen}; is it seated?")
        elif e.type == "tool_detach":
            time.sleep(0.15)
            if link.send({"cmd": "status"})["tool_ohms"][ARM_INDEX[d["arm"]] - 1] is not None:
                raise MachineError(f"{d['arm']} released {d['tool']} but it is still on the coupler")
        elif e.type == "probe":
            time.sleep(0.2)
            _check_events(link, touched)
            if time.monotonic() - touched.get(d["arm"], -1e9) > 1.5:
                raise MachineError(f"probe on {d['arm']} never touched at {d['at']}: part missing or out of position?")
        elif e.type == "inspect" and inspector is not None:
            if not inspector(d):
                raise MachineError(f"inspection failed: {d.get('check')} at {d.get('at')}")
        elif e.type == "flip":
            flips += 1
            link.send({"cmd": "aux", "name": "flip", "deg": 180 if flips % 2 else 0, "ms": int(d["dur"] * 1000)})
        elif e.type == "flash":
            t0 = time.monotonic()
            link.send({"cmd": "aux", "name": "dock", "deg": 160, "ms": 800})  # push the USB-C plug in
            time.sleep(1.0)
            try:
                if flasher is not None and not flasher(d):
                    raise MachineError(f"flashing {d['firmware']} or its self-test failed")
            finally:
                link.send({"cmd": "aux", "name": "dock", "deg": 0, "ms": 800})
                time.sleep(0.9)
            offset += max(0.0, time.monotonic() - t0 - d["dur"])
        elif e.type == "operator":
            t0 = time.monotonic()
            link.send({"cmd": "tower", "state": "wait"})
            log(f"OPERATOR: {d['message']}")
            touched.pop("_start", None)
            while True:
                _check_events(link, touched)
                if "_start" in touched:
                    st = link.send({"cmd": "status"})
                    if not st.get("door_open") and not st.get("curtain_broken"):
                        break
                    log("close the door and clear the window, then press START again")
                    touched.pop("_start", None)
                time.sleep(0.1)
            link.send({"cmd": "tower", "state": "run"})
            offset += (time.monotonic() - t0) - d["dur"]
    time.sleep(0.5)
    _check_events(link, touched)


def _check_events(link: SerialLink, touched: dict[str, float] | None = None) -> None:
    while not link.events.empty():
        ev = link.events.get()
        if ev["event"] in ("estop", "watchdog", "stop"):
            raise MachineError(f"machine stopped: {ev['event']}")
        if ev["event"] == "interlock":
            raise MachineError(f"safety interlock: {ev.get('source')} opened while running; close it and send reset")
        if ev["event"] == "start" and touched is not None:
            touched["_start"] = time.monotonic()
        if ev["event"] == "probe" and touched is not None:
            touched[f"arm{ev['arm']}"] = time.monotonic()
