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
) -> None:
    """Send every segment at its planned time.

    Sensor events pause the schedule until the ESP32 reports them, and inspect
    events call `inspector` (e.g. forge.vision) — a failed check aborts the run.
    """
    items = [(s.t0, 0, "move", s) for s in timeline.segments]
    items += [(e.t, 1, "event", e) for e in timeline.events]
    items.sort(key=lambda i: (i[0], i[1]))

    offset = 0.0  # seconds we have slipped behind the plan (waiting on sensors)
    t_start = time.monotonic()
    for t, _, kind, obj in items:
        wait = t_start + t + offset - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        _check_events(link)
        if kind == "move":
            link.send({"cmd": "move", "arm": ARM_INDEX[obj.arm], "servo": obj.servos, "ms": int(obj.dur * 1000)})
            continue
        e = obj
        if e.type == "step":
            log(f"[{t:6.2f}s] step {e.data['index']}: {e.data['label']}")
        elif e.type == "pump":
            link.send({"cmd": "aux", "ch": "pump", "on": e.data["on"]})
        elif e.type == "conveyor":
            link.send({"cmd": "aux", "ch": "conveyor", "on": True})
        elif e.type == "sensor":
            waited = _wait_for_ir(link, timeout=10.0)
            link.send({"cmd": "aux", "ch": "conveyor", "on": False})
            offset += waited
        elif e.type == "inspect" and inspector is not None:
            if not inspector(e.data):
                raise MachineError(f"inspection failed: {e.data.get('check')} at {e.data.get('at')}")
    time.sleep(0.5)
    _check_events(link)


def _check_events(link: SerialLink) -> None:
    while not link.events.empty():
        ev = link.events.get()
        if ev["event"] in ("estop", "watchdog"):
            raise MachineError(f"machine stopped: {ev['event']}")


def _wait_for_ir(link: SerialLink, timeout: float) -> float:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            ev = link.events.get(timeout=0.05)
        except queue.Empty:
            continue
        if ev["event"] == "ir" and ev.get("state") == 1:
            return max(0.0, time.monotonic() - t0)
        if ev["event"] in ("estop", "watchdog"):
            raise MachineError(f"machine stopped: {ev['event']}")
    raise MachineError("part never reached the IR sensor (conveyor jam?)")
