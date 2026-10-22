"""Turns recipe steps into a timed, collision-checked timeline.

The same Timeline drives the 3D simulation (exported as JSON) and the real
machine (played by forge.executor). Nothing reaches the servos unless the
whole recipe plans cleanly first — this is the "digital twin" check.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

from .config import Arm, Cell
from .kinematics import Unreachable, forward, inverse

WAYPOINT_MM = 20.0  # long moves are split so the tool travels in a straight line
TOOL_MS = 300


class PlanError(Exception):
    pass


@dataclass
class Segment:
    arm: str
    t0: float
    dur: float
    start: list[float]
    end: list[float]
    servos: list[float]
    tip: list[float]


@dataclass
class Event:
    t: float
    type: str
    data: dict = field(default_factory=dict)


@dataclass
class Timeline:
    segments: list[Segment] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)
    duration: float = 0.0

    def to_dict(self) -> dict:
        return {
            "duration": round(self.duration, 3),
            "segments": [asdict(s) for s in self.segments],
            "events": [asdict(e) for e in self.events],
        }


@dataclass
class ArmState:
    joints: list[float]
    tip: tuple[float, float, float]
    t: float = 0.0
    holding: str | None = None


class Planner:
    def __init__(self, cell: Cell, arms: list[str] | None = None):
        self.cell = cell
        self.arm_names = arms or list(cell.arms)
        self.timeline = Timeline()
        self.items: dict[str, dict] = {}
        self.state: dict[str, ArmState] = {}
        for name in self.arm_names:
            arm = cell.arms[name]
            joints = inverse(cell, arm, cell.park[name], cell.motion["tool_pitch"], self._tool_open(arm))
            self.state[name] = ArmState(joints=joints, tip=cell.park[name])
        self.clock = 0.0

    # ---------- helpers ----------
    def _arm(self, name: str) -> Arm:
        if name not in self.state:
            raise PlanError(f"arm '{name}' is not enabled in this recipe")
        return self.cell.arms[name]

    def _tool_open(self, arm: Arm) -> float:
        return float(self.cell.tools[arm.tool]["open"])

    def _tool_closed(self, arm: Arm) -> float:
        return float(self.cell.tools[arm.tool]["closed"])

    def _event(self, t: float, type_: str, **data) -> None:
        self.timeline.events.append(Event(round(t, 3), type_, data))

    def _push(self, arm: Arm, joints: list[float], dur: float, tip) -> None:
        st = self.state[arm.name]
        self.timeline.segments.append(
            Segment(
                arm=arm.name,
                t0=round(st.t, 3),
                dur=round(dur, 3),
                start=list(st.joints),
                end=list(joints),
                servos=self.cell.joints_to_servos(arm, joints),
                tip=[round(c, 1) for c in tip],
            )
        )
        st.joints, st.tip, st.t = list(joints), tuple(tip), st.t + dur

    def move(self, arm: Arm, target, tool: float | None = None) -> None:
        """Straight-line cartesian move of the tool tip, split into waypoints."""
        st = self.state[arm.name]
        target = tuple(float(c) for c in target)
        tool = st.joints[4] if tool is None else tool
        try:  # check the destination first so errors name the real target
            inverse(self.cell, arm, target, self.cell.motion["tool_pitch"], tool)
        except Unreachable as e:
            raise PlanError(str(e)) from None
        dist = math.dist(st.tip, target)
        steps = max(1, math.ceil(dist / WAYPOINT_MM))
        total_ms = max(self.cell.motion["min_move_ms"], 1000 * dist / self.cell.motion["speed_mm_s"])
        start = st.tip
        for i in range(1, steps + 1):
            f = i / steps
            p = tuple(start[k] + (target[k] - start[k]) * f for k in range(3))
            try:
                joints = inverse(self.cell, arm, p, self.cell.motion["tool_pitch"], tool)
            except Unreachable as e:
                raise PlanError(str(e)) from None
            self._push(arm, joints, total_ms / steps / 1000, p)

    def set_tool(self, arm: Arm, value: float) -> None:
        st = self.state[arm.name]
        joints = list(st.joints)
        joints[4] = value
        self._push(arm, joints, TOOL_MS / 1000, st.tip)

    def above(self, loc) -> tuple[float, float, float]:
        x, y, z = loc
        return x, y, z + self.cell.motion["approach_height"]

    def park(self, arm: Arm) -> None:
        st = self.state[arm.name]
        self.move(arm, (st.tip[0], st.tip[1], max(st.tip[2], self.cell.park[arm.name][2])))
        self.move(arm, self.cell.park[arm.name])

    # ---------- actions ----------
    def a_pick(self, arm: Arm, item: str, frm=None, **_):
        st = self.state[arm.name]
        if st.holding:
            raise PlanError(f"{arm.name} is already holding {st.holding}")
        if item not in self.items:
            raise PlanError(f"unknown item '{item}'")
        loc = self.cell.location(frm) if frm is not None else tuple(self.items[item]["at"])
        if math.dist(loc[:2], self.items[item]["at"][:2]) > 3:
            raise PlanError(f"{item} is not at {frm}")
        self.move(arm, self.above(loc), self._tool_open(arm))
        self.move(arm, loc)
        self.set_tool(arm, self._tool_closed(arm))
        if arm.tool == "vacuum":
            self._event(st.t, "pump", arm=arm.name, on=True)
        self._event(st.t, "attach", arm=arm.name, item=item)
        st.holding = item
        self.move(arm, self.above(loc))

    def a_place(self, arm: Arm, to, into=None, stay=False, **_):
        st = self.state[arm.name]
        if not st.holding:
            raise PlanError(f"{arm.name} is not holding anything to place")
        loc = self.cell.location(to)
        self.move(arm, self.above(loc))
        self.move(arm, loc)
        self.set_tool(arm, self._tool_open(arm))
        if arm.tool == "vacuum":
            self._event(st.t, "pump", arm=arm.name, on=False)
        self._event(st.t, "detach", arm=arm.name, item=st.holding, pos=list(loc), into=into)
        self.items[st.holding]["at"] = list(loc)
        st.holding = None
        self.move(arm, self.above(loc))
        if not stay:
            self.park(arm)

    def a_hold(self, arm: Arm, at, **_):
        loc = self.cell.location(at)
        self.move(arm, self.above(loc), self._tool_open(arm))
        self.move(arm, loc)
        self.set_tool(arm, self._tool_closed(arm))
        self._event(self.state[arm.name].t, "hold", arm=arm.name, on=True)

    def a_release(self, arm: Arm, **_):
        st = self.state[arm.name]
        self.set_tool(arm, self._tool_open(arm))
        self._event(st.t, "hold", arm=arm.name, on=False)
        self.move(arm, self.above(st.tip))
        self.park(arm)

    def a_press(self, arm: Arm, at, depth=2, **_):
        loc = self.cell.location(at)
        self.move(arm, self.above(loc))
        self.move(arm, loc)
        self.move(arm, (loc[0], loc[1], loc[2] - float(depth)))
        self._event(self.state[arm.name].t, "press", arm=arm.name, at=list(loc))
        self.move(arm, loc)
        self.move(arm, self.above(loc))
        self.park(arm)

    def a_inspect(self, arm: Arm, at, check="present", **_):
        loc = self.cell.location(at)
        st = self.state[arm.name]
        self.move(arm, (loc[0], loc[1], loc[2] + 50))
        self._push(arm, list(st.joints), 0.8, st.tip)  # dwell while the camera looks
        self._event(st.t, "inspect", arm=arm.name, at=list(loc), check=check, result="pass")
        self.park(arm)

    def a_move(self, arm: Arm, to, **_):
        self.move(arm, self.cell.location(to))

    def a_park(self, arm: Arm, **_):
        self.park(arm)

    READY_POSE = [0.0, 160.0, -90.0, 0.0]  # upright; every +/-20 deg sweep stays out of the centre zone

    def a_calibrate(self, arm: Arm, sweep=20, **_):
        """Raise to the ready pose and sweep each joint +/- sweep degrees (week 1 check)."""
        st = self.state[arm.name]
        park = list(st.joints)
        ready = self.READY_POSE + [park[4]]
        self._push(arm, ready, 0.8, forward(self.cell, arm, ready))
        for j in range(5):
            for delta in (sweep, -sweep, 0):
                joints = list(ready)
                joints[j] += delta
                self._push(arm, joints, 0.4, forward(self.cell, arm, joints))
        self._push(arm, park, 0.8, forward(self.cell, arm, park))
        self._event(st.t, "calibrated", arm=arm.name)

    # ---------- cell-level actions (no arm) ----------
    def c_conveyor(self, item, frm="conveyor_start", to="input", **_):
        a, b = self.cell.location(frm), self.cell.location(to)
        dur = math.dist(a, b) / self.cell.conveyor["speed_mm_s"]
        self.items.setdefault(item, {"at": list(a)})
        self._event(self.clock, "conveyor", item=item, frm=list(a), to=list(b), dur=round(dur, 3))
        self.items[item]["at"] = list(b)
        self._event(self.clock + dur, "sensor", name="ir_input", item=item)
        return dur

    def c_wait(self, ms, **_):
        return float(ms) / 1000

    # ---------- driver ----------
    ARM_ACTIONS = {"pick", "place", "hold", "release", "press", "inspect", "move", "park", "calibrate"}
    CELL_ACTIONS = {"conveyor", "wait"}

    def _run_one(self, action: str, params: dict, start: float) -> float:
        params = {("frm" if k == "from" else k): v for k, v in (params or {}).items()}
        if action in self.CELL_ACTIONS:
            return start + getattr(self, f"c_{action}")(**params)
        if action not in self.ARM_ACTIONS:
            raise PlanError(f"unknown action '{action}'")
        arm = self._arm(params.pop("arm", None) or "")
        st = self.state[arm.name]
        st.t = max(st.t, start)
        getattr(self, f"a_{action}")(arm, **params)
        return st.t

    def plan(self, recipe: dict) -> Timeline:
        for name, spec in (recipe.get("items") or {}).items():
            if "at" in spec:
                pos = list(self.cell.location(spec["at"]))
                self.items[name] = {"at": pos}
                self._event(0, "spawn", item=name, pos=pos, kind=spec.get("kind", name))
            else:
                self._event(0, "declare", item=name, kind=spec.get("kind", name))

        for idx, step in enumerate(recipe["steps"], 1):
            step = dict(step)
            note = step.pop("note", None)
            if len(step) != 1:
                raise PlanError(f"step {idx}: expected exactly one action, got {list(step)}")
            (action, params), = step.items()
            label = note or _describe(action, params)
            self._event(self.clock, "step", index=idx, label=label)
            try:
                if action == "parallel":
                    used = [p.get("arm") for sub in params for p in sub.values() if isinstance(p, dict)]
                    if len([u for u in used if u]) != len({u for u in used if u}):
                        raise PlanError("an arm appears twice in one parallel block")
                    ends = [self._run_one(*next(iter(sub.items())), self.clock) for sub in params]
                    self.clock = max(ends)
                else:
                    self.clock = self._run_one(action, params, self.clock)
            except (PlanError, KeyError) as e:
                msg = e.args[0] if e.args else str(e)
                raise PlanError(f"step {idx} ({label}): {msg}") from None
            # holders keep their pose; everyone else must be done before the next step
            self.clock = max([self.clock] + [s.t for s in self.state.values()])

        self.timeline.duration = round(self.clock, 3)
        self.timeline.segments.sort(key=lambda s: s.t0)
        self.timeline.events.sort(key=lambda e: e.t)
        check_zone(self.cell, self.timeline)
        return self.timeline


def _describe(action: str, params) -> str:
    if action == "parallel":
        return " + ".join(_describe(*next(iter(s.items()))) for s in params)
    p = params or {}
    who = p.get("arm", "cell")
    what = p.get("item") or p.get("to") or p.get("at") or p.get("from") or ""
    return f"{who}: {action} {what}".strip()


def in_zone(cell: Cell, tip) -> bool:
    cx, cy = cell.zone["center"]
    return math.hypot(tip[0] - cx, tip[1] - cy) <= cell.zone["radius"]


def zone_intervals(cell: Cell, timeline: Timeline) -> list[tuple[float, float, str]]:
    """(enter, leave, arm) for every visit of an arm to the shared centre zone."""
    out = []
    for arm in {s.arm for s in timeline.segments}:
        segs = [s for s in timeline.segments if s.arm == arm]
        inside_since = None
        for s in segs:
            if in_zone(cell, s.tip) and inside_since is None:
                inside_since = s.t0
            elif not in_zone(cell, s.tip) and inside_since is not None:
                out.append((inside_since, s.t0 + s.dur, arm))
                inside_since = None
        if inside_since is not None:
            out.append((inside_since, timeline.duration, arm))
    return out


def check_zone(cell: Cell, timeline: Timeline) -> None:
    visits = zone_intervals(cell, timeline)
    cap = cell.zone["capacity"]
    for t, _, _ in visits:
        present = sorted(a for a0, a1, a in visits if a0 <= t < a1)
        if len(present) > cap:
            raise PlanError(
                f"zone conflict at t={t:.2f}s: {', '.join(present)} are all in the centre zone "
                f"(capacity {cap}). Put these steps in sequence instead of in parallel."
            )
