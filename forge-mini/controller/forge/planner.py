"""Turns recipe steps into a timed, collision-checked timeline.

The same Timeline drives the 3D simulation (exported as JSON) and the real
machine (played by forge.executor). Nothing reaches the servos unless the
whole recipe plans cleanly first: this is the "digital twin" check.

Every arm starts bare (just the tool-changer coupler). Tools live in two racks
and the spindle's bits live in a bit block. The planner tracks where every
tool, bit and part is, uses the mounted tool's length for the kinematics, and
refuses actions the mounted tool cannot do.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

from .config import Arm, Cell
from .kinematics import Unreachable, forward, inverse

WAYPOINT_MM = 20.0  # long moves are split so the TCP travels in a straight line
SIGNAL_MS = 300
LOCK_MS = 400

# Which tool types can do which action.
NEEDS = {
    "pick": ("gripper", "vacuum"),
    "place": ("gripper", "vacuum"),
    "hold": ("clamp", "gripper"),
    "machine": ("spindle",),
    "draw": ("pen", "dispenser"),
    "probe": ("probe",),
}


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
    initial: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "duration": round(self.duration, 3),
            "initial": self.initial,
            "segments": [asdict(s) for s in self.segments],
            "events": [asdict(e) for e in self.events],
        }


@dataclass
class ArmState:
    joints: list[float]
    tip: tuple[float, float, float]
    t: float = 0.0
    holding: str | None = None
    tool: str | None = None
    bit: str | None = None


class Planner:
    def __init__(self, cell: Cell, arms: list[str] | None = None):
        self.cell = cell
        self.timeline = Timeline()
        self.items: dict[str, dict] = {}
        self.tool_at = {name: "rack" for name in cell.tools}
        self.bit_at = {name: "block" for name in cell.bits}
        self.state: dict[str, ArmState] = {}
        for name in arms or list(cell.arms):
            if name not in cell.arms:
                raise PlanError(f"unknown arm '{name}'")
            arm = cell.arms[name]
            park = cell.park[name]
            joints = inverse(cell, arm, park, self.pitch, cell.lock["open"])
            self.state[name] = ArmState(joints=joints, tip=park)
            self.timeline.initial[name] = list(joints)
        self.clock = 0.0

    @property
    def pitch(self) -> float:
        return self.cell.motion["tool_pitch"]

    # ---------- helpers ----------
    def _arm(self, name: str) -> Arm:
        if name not in self.state:
            raise PlanError(f"arm '{name}' is not enabled in this recipe")
        return self.cell.arms[name]

    def tool_len(self, st: ArmState) -> float:
        """Coupler face -> TCP for whatever is mounted now."""
        n = 0.0
        if st.tool:
            n += self.cell.tools[st.tool].length
        if st.bit:
            n += self.cell.bits[st.bit].stickout
        return n

    def l3(self, st: ArmState) -> float:
        return self.cell.links.coupler + self.tool_len(st)

    def flange(self, st: ArmState) -> tuple[float, float, float]:
        x, y, z = st.tip
        return x, y, z + self.tool_len(st)

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

    def _ik(self, arm: Arm, target, lock: float) -> list[float]:
        try:
            return inverse(self.cell, arm, target, self.pitch, lock, self.l3(self.state[arm.name]))
        except Unreachable as e:
            raise PlanError(str(e)) from None

    def move(self, arm: Arm, target, speed: float | None = None) -> None:
        """Straight-line move of the TCP, split into waypoints, each one IK-checked."""
        st = self.state[arm.name]
        target = tuple(float(c) for c in target)
        lock = st.joints[4]
        self._ik(arm, target, lock)  # check the destination first so errors name the real target
        dist = math.dist(st.tip, target)
        if dist < 1e-6:
            return
        speed = speed or self.cell.motion["speed_mm_s"]
        steps = max(1, math.ceil(dist / WAYPOINT_MM))
        total = max(self.cell.motion["min_move_ms"] / 1000, dist / speed)
        start = st.tip
        for i in range(1, steps + 1):
            f = i / steps
            p = tuple(start[k] + (target[k] - start[k]) * f for k in range(3))
            self._push(arm, self._ik(arm, p, lock), total / steps, p)

    def move_flange(self, arm: Arm, flange) -> None:
        x, y, z = flange
        self.move(arm, (x, y, z - self.tool_len(self.state[arm.name])))

    def dwell(self, arm: Arm, seconds: float) -> None:
        st = self.state[arm.name]
        self._push(arm, list(st.joints), seconds, st.tip)

    def set_lock(self, arm: Arm, value: float) -> None:
        st = self.state[arm.name]
        joints = list(st.joints)
        joints[4] = value
        self._push(arm, joints, LOCK_MS / 1000, st.tip)

    def signal(self, arm: Arm, state: str) -> None:
        """Drive the tool-signal channel (gripper jaw, pump, spindle speed, glue plunger)."""
        st = self.state[arm.name]
        tool = self.cell.tools[st.tool]
        value = tool.signal.get(state, 0)
        self._event(st.t, "signal", arm=arm.name, tool=st.tool, state=state, value=value)
        self.dwell(arm, SIGNAL_MS / 1000)

    def above(self, loc) -> tuple[float, float, float]:
        x, y, z = loc
        return x, y, z + self.cell.motion["approach_height"]

    def park(self, arm: Arm) -> None:
        """Rise, travel to the park XY, settle. Park is defined for the coupler face."""
        st = self.state[arm.name]
        px, py, pz = self.cell.park[arm.name]
        fx, fy, fz = self.flange(st)
        travel = max(fz, pz)
        self.move_flange(arm, (fx, fy, travel))
        self.move_flange(arm, (px, py, travel))
        self.move_flange(arm, (px, py, pz))

    def need(self, arm: Arm, action: str) -> None:
        st = self.state[arm.name]
        kinds = NEEDS[action]
        have = self.cell.tools[st.tool].type if st.tool else None
        if have in kinds:
            if action == "machine" and not st.bit:
                raise PlanError(f"{arm.name} has the spindle but no bit; add bit_change: {{arm: {arm.name}, bit: ...}}")
            return
        options = [n for n, t in self.cell.tools.items() if t.type in kinds and arm.name in self.cell.racks[t.rack]["arms"]]
        hint = f"add tool_change: {{arm: {arm.name}, tool: {options[0]}}} first" if options else (
            f"no {'/'.join(kinds)} tool sits in a rack {arm.name} can reach"
        )
        raise PlanError(f"{arm.name} needs a {'/'.join(kinds)} tool to {action} but holds {st.tool or 'no tool'}; {hint}")

    # ---------- tool changer ----------
    def _return_tool(self, arm: Arm) -> None:
        st = self.state[arm.name]
        if st.holding:
            raise PlanError(f"{arm.name} is still holding {st.holding}; place it before changing tools")
        if st.bit:
            self.a_bit_change(arm, None)
        tool = self.cell.tools[st.tool]
        rack_z = self.cell.racks[tool.rack]["z"]
        lifted = rack_z + tool.length + 5
        fx, fy, fz = self.flange(st)
        self.move_flange(arm, (fx, fy, max(fz, lifted)))
        self.move_flange(arm, (*tool.slot, lifted))
        self.move_flange(arm, (*tool.slot, rack_z))
        self.set_lock(arm, self.cell.lock["open"])
        self._event(st.t, "tool_detach", arm=arm.name, tool=tool.name, slot=list(tool.slot), z=rack_z)
        self.tool_at[tool.name] = "rack"
        x, y, z = st.tip
        st.tool, st.tip = None, (x, y, z + tool.length)  # same flange, bare TCP
        self.move_flange(arm, (*tool.slot, rack_z + self.cell.motion["approach_height"]))

    def a_tool_change(self, arm: Arm, tool=None, **_):
        st = self.state[arm.name]
        tool = None if tool in (None, "none") else tool
        if tool == st.tool:
            return
        if tool is not None:
            if tool not in self.cell.tools:
                raise PlanError(f"unknown tool '{tool}' (known: {', '.join(self.cell.tools)})")
            spec = self.cell.tools[tool]
            rack = self.cell.racks[spec.rack]
            if arm.name not in rack["arms"]:
                users = " or ".join(rack["arms"])
                raise PlanError(f"{arm.name} cannot reach {spec.rack}, where {tool} lives; use {users}")
            if self.tool_at[tool] != "rack":
                raise PlanError(f"{tool} is on {self.tool_at[tool]}; return it first with tool_change: {{arm: {self.tool_at[tool]}, tool: none}}")
        if st.tool:
            self._return_tool(arm)
        if tool is None:
            self.park(arm)
            return
        rack_z = rack["z"]
        above = rack_z + self.cell.motion["approach_height"]
        fx, fy, fz = self.flange(st)
        self.move_flange(arm, (fx, fy, max(fz, above)))
        self.move_flange(arm, (*spec.slot, above))
        self.move_flange(arm, (*spec.slot, rack_z))
        self.set_lock(arm, self.cell.lock["locked"])
        st.tool = tool
        x, y, z = st.tip
        st.tip = (x, y, z - spec.length)  # same flange, TCP now at the tool tip
        self.tool_at[tool] = arm.name
        self._event(st.t, "tool_attach", arm=arm.name, tool=tool)
        self._event(st.t, "id_check", arm=arm.name, tool=tool, type=spec.type, ohms=spec.id_ohms)
        self.move_flange(arm, (*spec.slot, rack_z + spec.length + 5))
        self.park(arm)

    def a_bit_change(self, arm: Arm, bit=None, **_):
        st = self.state[arm.name]
        bit = None if bit in (None, "none") else bit
        if bit == st.bit:
            return
        if not st.tool or self.cell.tools[st.tool].type != "spindle":
            raise PlanError(f"{arm.name} needs the spindle mounted to change bits")
        block = self.cell.bit_block
        if arm.name not in block["arms"]:
            raise PlanError(f"{arm.name} cannot reach the bit block; use {' or '.join(block['arms'])}")
        collet = block["z"] + 8
        spindle_len = self.cell.tools[st.tool].length
        if st.bit:  # put the current bit back: lower it into its hole, slide off the magnet
            old = self.cell.bits[st.bit]
            lifted = collet + old.stickout + 5
            fx, fy, fz = self.flange(st)
            self.move_flange(arm, (fx, fy, max(fz, lifted + spindle_len)))
            self.move(arm, (*old.slot, lifted - old.stickout - 0 + 0))
            self.move(arm, (*old.slot, collet - old.stickout))
            sx, sy = block["strip"]
            self.move(arm, (old.slot[0] + sx, old.slot[1] + sy, collet - old.stickout), speed=20)
            self._event(st.t, "bit_detach", arm=arm.name, bit=old.name, slot=list(old.slot), z=collet)
            self.bit_at[old.name] = "block"
            x, y, z = st.tip
            st.bit, st.tip = None, (x, y, z + old.stickout)
            self.move(arm, (x, y, collet + self.cell.motion["approach_height"]))
        if bit is None:
            self.park(arm)
            return
        if bit not in self.cell.bits:
            raise PlanError(f"unknown bit '{bit}' (known: {', '.join(self.cell.bits)})")
        new = self.cell.bits[bit]
        fx, fy, fz = self.flange(st)
        self.move_flange(arm, (fx, fy, max(fz, collet + 30 + spindle_len)))
        self.move(arm, (*new.slot, collet + 30))
        self.move(arm, (*new.slot, collet), speed=30)
        self._event(st.t, "bit_attach", arm=arm.name, bit=bit)
        self.bit_at[bit] = arm.name
        x, y, z = st.tip
        st.bit, st.tip = bit, (x, y, z - new.stickout)
        self.move(arm, (*new.slot, collet + 5))
        self.park(arm)

    # ---------- part handling ----------
    def a_pick(self, arm: Arm, item: str, frm=None, **_):
        self.need(arm, "pick")
        st = self.state[arm.name]
        if st.holding:
            raise PlanError(f"{arm.name} is already holding {st.holding}")
        if item not in self.items:
            raise PlanError(f"unknown item '{item}'")
        loc = self.cell.location(frm) if frm is not None else tuple(self.items[item]["at"])
        if math.dist(loc[:2], self.items[item]["at"][:2]) > 3:
            raise PlanError(f"{item} is not at {frm}")
        gripper = self.cell.tools[st.tool].type == "gripper"
        self.move(arm, self.above(loc))
        if gripper:
            self.signal(arm, "open")
        self.move(arm, loc, speed=40)
        self.signal(arm, "closed" if gripper else "on")
        self._event(st.t, "attach", arm=arm.name, item=item)
        st.holding = item
        self.move(arm, self.above(loc))

    def a_place(self, arm: Arm, to, into=None, press=0, stay=False, **_):
        self.need(arm, "place")
        st = self.state[arm.name]
        if not st.holding:
            raise PlanError(f"{arm.name} is not holding anything to place")
        loc = self.cell.location(to)
        gripper = self.cell.tools[st.tool].type == "gripper"
        self.move(arm, self.above(loc))
        self.move(arm, loc, speed=40)
        if press:
            self.move(arm, (loc[0], loc[1], loc[2] - float(press)), speed=5)
            self._event(st.t, "press", arm=arm.name, at=list(loc), depth=float(press))
        self.signal(arm, "open" if gripper else "off")
        self._event(st.t, "detach", arm=arm.name, item=st.holding, pos=list(loc), into=into)
        self.items[st.holding]["at"] = list(loc)
        st.holding = None
        self.move(arm, self.above(loc))
        if not stay:
            self.park(arm)

    def a_hold(self, arm: Arm, at, **_):
        self.need(arm, "hold")
        st = self.state[arm.name]
        loc = self.cell.location(at)
        self.move(arm, self.above(loc))
        if self.cell.tools[st.tool].type == "gripper":
            self.signal(arm, "open")
        self.move(arm, loc, speed=40)
        self.move(arm, (loc[0], loc[1], loc[2] - 1), speed=5)  # 1 mm preload
        if self.cell.tools[st.tool].type == "gripper":
            self.signal(arm, "closed")
        self._event(st.t, "hold", arm=arm.name, on=True)

    def a_release(self, arm: Arm, **_):
        st = self.state[arm.name]
        if st.tool and self.cell.tools[st.tool].type == "gripper":
            self.signal(arm, "open")
        self._event(st.t, "hold", arm=arm.name, on=False)
        self.move(arm, self.above(st.tip))
        self.park(arm)

    # ---------- processes ----------
    def a_machine(self, arm: Arm, at, depth=3, **_):
        """Ream, drill, countersink or drive at one or more locations with the mounted bit."""
        self.need(arm, "machine")
        st = self.state[arm.name]
        bit = self.cell.bits[st.bit]
        feed = self.cell.motion["feed_mm_s"]
        holes = at if isinstance(at, list) and at and not isinstance(at[0], (int, float)) else [at]
        speed_state = "fast" if bit.kind == "drill" else "slow"
        self.move(arm, self.above(self.cell.location(holes[0])))
        self.signal(arm, speed_state)
        for ref in holes:
            x, y, z = self.cell.location(ref)
            self.move(arm, (x, y, z + 3))
            t0 = st.t
            self.move(arm, (x, y, z - float(depth)), speed=feed)
            self._event(t0, "cut", arm=arm.name, at=[x, y, z], bit=bit.name, kind=bit.kind, dia=bit.dia,
                        depth=float(depth), dur=round(st.t - t0, 3))
            self.move(arm, (x, y, z + 3), speed=feed * 3)
            self.move(arm, self.above((x, y, z)))
        self.signal(arm, "off")
        self.park(arm)

    def a_draw(self, arm: Arm, at, path, **_):
        """Pen or glue along a path of [dx, dy] offsets from `at` (mm)."""
        self.need(arm, "draw")
        st = self.state[arm.name]
        ox, oy, oz = self.cell.location(at)
        pts = [(ox + float(dx), oy + float(dy), oz) for dx, dy in path]
        glue = self.cell.tools[st.tool].type == "dispenser"
        self.move(arm, self.above(pts[0]))
        self.move(arm, pts[0], speed=30)
        if glue:
            self.signal(arm, "push")
        for a, b in zip(pts, pts[1:]):
            t0 = st.t
            self.move(arm, b, speed=25)
            self._event(t0, "stroke", arm=arm.name, frm=list(a), to=list(b), dur=round(st.t - t0, 3),
                        ink="glue" if glue else "pen")
        if glue:
            self.signal(arm, "stop")
        self.move(arm, self.above(pts[-1]))
        self.park(arm)

    def a_probe(self, arm: Arm, at, expect=None, tol=0.3, **_):
        """Touch down on each location and record the height (checks pins, holes, flatness)."""
        self.need(arm, "probe")
        st = self.state[arm.name]
        refs = at if isinstance(at, list) and at and not isinstance(at[0], (int, float)) else [at]
        for ref in refs:
            x, y, z = self.cell.location(ref)
            self.move(arm, self.above((x, y, z)))
            self.move(arm, (x, y, z + 3))
            self.move(arm, (x, y, z), speed=5)
            measured = z if expect is None else float(expect)
            self._event(st.t, "probe", arm=arm.name, at=[x, y, z], measured=measured, expect=expect, tol=tol, ok=True)
            self.move(arm, self.above((x, y, z)))
        self.park(arm)

    def a_move(self, arm: Arm, to, **_):
        self.move(arm, self.cell.location(to))

    def a_park(self, arm: Arm, **_):
        self.park(arm)

    READY_POSE = [0.0, 160.0, -90.0, 0.0]  # upright; every +/-20 deg sweep stays out of the centre zone

    def a_calibrate(self, arm: Arm, sweep=20, **_):
        """Raise to the ready pose and sweep each joint +/- sweep degrees (week 1 check)."""
        st = self.state[arm.name]
        l3 = self.l3(st)
        park = list(st.joints)
        ready = self.READY_POSE + [park[4]]
        self._push(arm, ready, 0.8, forward(self.cell, arm, ready, l3))
        for j in range(4):
            for delta in (sweep, -sweep, 0):
                joints = list(ready)
                joints[j] += delta
                self._push(arm, joints, 0.4, forward(self.cell, arm, joints, l3))
        self._push(arm, park, 0.8, forward(self.cell, arm, park, l3))
        self._event(st.t, "calibrated", arm=arm.name)

    # ---------- cell-level actions (no arm) ----------
    def c_inspect(self, at, check="present", **_):
        """Overhead camera check."""
        self._event(self.clock, "inspect", at=list(self.cell.location(at)), check=check, result="pass")
        return 0.8

    def c_wait(self, ms, **_):
        return float(ms) / 1000

    # ---------- driver ----------
    ARM_ACTIONS = {"pick", "place", "hold", "release", "machine", "draw", "probe", "move", "park",
                   "calibrate", "tool_change", "bit_change"}
    CELL_ACTIONS = {"inspect", "wait"}

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

    def _run_seq(self, sub, start: float) -> float:
        """A parallel branch is one action, or a list of actions run in order."""
        t = start
        for step in sub if isinstance(sub, list) else [sub]:
            (action, params), = step.items()
            t = self._run_one(action, dict(params or {}), t)
        return t

    @staticmethod
    def _arms_in(sub) -> set[str]:
        steps = sub if isinstance(sub, list) else [sub]
        return {p.get("arm") for s in steps for p in s.values() if isinstance(p, dict) and p.get("arm")}

    def plan(self, recipe: dict) -> Timeline:
        for name, spec in (recipe.get("items") or {}).items():
            pos = list(self.cell.location(spec["at"]))
            self.items[name] = {"at": pos}
            self._event(0, "spawn", item=name, pos=pos, kind=spec.get("kind", name))

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
                    seen: set[str] = set()
                    for sub in params:
                        arms = self._arms_in(sub)
                        if arms & seen:
                            raise PlanError(f"{', '.join(sorted(arms & seen))} appears in two branches of one parallel block")
                        seen |= arms
                    ends = [self._run_seq(sub, self.clock) for sub in params]
                    self.clock = max(ends)
                else:
                    self.clock = self._run_one(action, params, self.clock)
            except (PlanError, KeyError) as e:
                msg = e.args[0] if e.args else str(e)
                raise PlanError(f"step {idx} ({label}): {msg}") from None
            self.clock = max([self.clock] + [s.t for s in self.state.values()])

        self.timeline.duration = round(self.clock, 3)
        self.timeline.segments.sort(key=lambda s: s.t0)
        self.timeline.events.sort(key=lambda e: e.t)
        check_zones(self.cell, self.timeline)
        return self.timeline


def _describe(action: str, params) -> str:
    if action == "parallel":
        parts = []
        for sub in params:
            for s in sub if isinstance(sub, list) else [sub]:
                parts.append(_describe(*next(iter(s.items()))))
        return " + ".join(parts)
    p = params or {}
    who = p.get("arm", "cell")
    what = p.get("tool") or p.get("bit") or p.get("item") or p.get("to") or p.get("at") or p.get("from") or ""
    if isinstance(what, list):
        what = ", ".join(map(str, what))
    return f"{who}: {action.replace('_', ' ')} {what}".strip()


def in_zone(zone: dict, tip) -> bool:
    cx, cy = zone["center"]
    return math.hypot(tip[0] - cx, tip[1] - cy) <= zone["radius"]


def zone_intervals(zone: dict, timeline: Timeline) -> list[tuple[float, float, str]]:
    """(enter, leave, arm) for every visit of an arm to one shared zone."""
    out = []
    for arm in sorted({s.arm for s in timeline.segments}):
        inside_since = None
        for s in (s for s in timeline.segments if s.arm == arm):
            inside = in_zone(zone, s.tip)
            if inside and inside_since is None:
                inside_since = s.t0
            elif not inside and inside_since is not None:
                out.append((inside_since, s.t0 + s.dur, arm))
                inside_since = None
        if inside_since is not None:
            out.append((inside_since, timeline.duration, arm))
    return out


def check_zones(cell: Cell, timeline: Timeline) -> None:
    for name, zone in cell.zones.items():
        visits = zone_intervals(zone, timeline)
        cap = zone["capacity"]
        for t, _, _ in visits:
            present = sorted(a for a0, a1, a in visits if a0 <= t < a1)
            if len(present) > cap:
                raise PlanError(
                    f"zone conflict in {name} at t={t:.2f}s: {', '.join(present)} are inside at once "
                    f"(capacity {cap}). Put these steps in sequence instead of in parallel."
                )
