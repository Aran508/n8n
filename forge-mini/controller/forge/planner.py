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

import bisect
import copy
import math
from dataclasses import asdict, dataclass, field

from .config import Arm, Cell
from .kinematics import Unreachable, forward, inverse

WAYPOINT_MM = 20.0  # long moves are split so the TCP travels in a straight line
SIGNAL_MS = 300
LOCK_MS = 400

# Which tool types can do which action.
NEEDS = {
    "pick": ("gripper", "vacuum", "tweezers"),
    "place": ("gripper", "vacuum", "tweezers"),
    "hold": ("clamp", "gripper"),
    "machine": ("spindle",),
    "drive": ("spindle",),
    "draw": ("pen", "dispenser"),
    "probe": ("probe",),
    "solder": ("iron",),
}
SAMPLE_S = 0.05  # collision/zone check resolution


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
    stats: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "duration": round(self.duration, 3),
            "initial": self.initial,
            "stats": self.stats,
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
    hold_site: str | None = None


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
        # per arm: (t0, t1, tip_start, tip_end, tool_len) for collision checks
        self.track: dict[str, list[tuple]] = {name: [] for name in self.state}
        self.holds: dict[str, list[list[float]]] = {name: [] for name in self.state}

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
        self.track[arm.name].append((st.t, st.t + dur, tuple(st.tip), tuple(tip), self.tool_len(st)))
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

    def travel(self, loc) -> tuple[float, float, float]:
        """Height for sideways moves between work points: clears other arms' tools at the part."""
        x, y, z = loc
        m = self.cell.motion
        return x, y, max(z + m["approach_height"], m.get("travel_plane", 0))

    def park(self, arm: Arm) -> None:
        """Rise, travel to the park XY, settle. Park is defined for the coupler face."""
        st = self.state[arm.name]
        px, py, pz = self.cell.park[arm.name]
        fx, fy, fz = self.flange(st)
        travel = max(fz, pz)
        self.move_flange(arm, (fx, fy, travel))
        try:  # over the top when the wrist allows it...
            self._ik(arm, (px, py, travel - self.tool_len(st)), st.joints[4])
            self.move_flange(arm, (px, py, travel))
        except PlanError:
            pass  # ...otherwise glide down to park on the diagonal
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
        raise PlanError(f"{arm.name} needs a {' or '.join(kinds)} to {action} but holds {st.tool or 'no tool'}; {hint}")

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
        gripper = self.cell.tools[st.tool].type in ("gripper", "tweezers")
        self.move(arm, self.travel(loc))
        if gripper:
            self.signal(arm, "open")
        self.move(arm, loc, speed=40)
        self.signal(arm, "closed" if gripper else "on")
        self._event(st.t, "attach", arm=arm.name, item=item)
        st.holding = item
        self.move(arm, self.travel(loc))

    def a_place(self, arm: Arm, to, into=None, press=0, stay=False, **_):
        self.need(arm, "place")
        st = self.state[arm.name]
        if not st.holding:
            raise PlanError(f"{arm.name} is not holding anything to place")
        loc = self.cell.location(to)
        gripper = self.cell.tools[st.tool].type in ("gripper", "tweezers")
        self.move(arm, self.travel(loc))
        self.move(arm, loc, speed=40)
        if press:
            self.move(arm, (loc[0], loc[1], loc[2] - float(press)), speed=5)
            self._event(st.t, "press", arm=arm.name, at=list(loc), depth=float(press))
        self.signal(arm, "open" if gripper else "off")
        self._event(st.t, "detach", arm=arm.name, item=st.holding, pos=list(loc), into=into)
        self.items[st.holding]["at"] = list(loc)
        st.holding = None
        self.move(arm, self.travel(loc))
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
        self.holds[arm.name].append([st.t, math.inf])

    def a_release(self, arm: Arm, **_):
        st = self.state[arm.name]
        if st.tool and self.cell.tools[st.tool].type == "gripper":
            self.signal(arm, "open")
        self._event(st.t, "hold", arm=arm.name, on=False)
        if self.holds[arm.name] and self.holds[arm.name][-1][1] == math.inf:
            self.holds[arm.name][-1][1] = st.t
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
        self.move(arm, self.travel(self.cell.location(holes[0])))
        self.signal(arm, speed_state)
        for ref in holes:
            x, y, z = self.cell.location(ref)
            self.move(arm, self.travel((x, y, z)))
            self.move(arm, (x, y, z + 3))
            t0 = st.t
            self.move(arm, (x, y, z - float(depth)), speed=feed)
            self._event(t0, "cut", arm=arm.name, at=[x, y, z], bit=bit.name, kind=bit.kind, dia=bit.dia,
                        depth=float(depth), dur=round(st.t - t0, 3))
            self.move(arm, (x, y, z + 3), speed=feed * 3)
            self.move(arm, self.travel((x, y, z)))
        self.signal(arm, "off")
        self.park(arm)

    def a_drive(self, arm: Arm, at, depth=3, **_):
        """Drive screws with the hex bit (same motion as machining, slow speed)."""
        st = self.state[arm.name]
        self.need(arm, "drive")
        if not st.bit or self.cell.bits[st.bit].kind != "hex_driver":
            hexes = [n for n, b in self.cell.bits.items() if b.kind == "hex_driver"]
            raise PlanError(f"{arm.name} needs a hex bit to drive screws; add bit_change: {{arm: {arm.name}, bit: {hexes[0]}}}")
        self.a_machine(arm, at, depth)

    def a_solder(self, arm: Arm, at, dwell=0.8, **_):
        """Solder joints: touch the pad, preheat, feed solder, let it flow, lift."""
        self.need(arm, "solder")
        st = self.state[arm.name]
        joints = at if isinstance(at, list) and at and not isinstance(at[0], (int, float)) else [at]
        self.move(arm, self.travel(self.cell.location(joints[0])))
        self.signal(arm, "heat")
        for ref in joints:
            x, y, z = self.cell.location(ref)
            self.move(arm, self.travel((x, y, z)))
            self.move(arm, (x, y, z + 4))
            self.move(arm, (x, y, z + 0.5), speed=10)
            self.dwell(arm, float(dwell))          # preheat pad and lead
            self.signal(arm, "feed")
            self.dwell(arm, 0.4)
            self.signal(arm, "heat")
            self._event(st.t, "solder", arm=arm.name, at=[x, y, z])
            self.dwell(arm, 0.3)                   # let the joint flow
            self.move(arm, (x, y, z + 6), speed=30)
            self.move(arm, self.travel((x, y, z)))
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
            self.move(arm, self.travel((x, y, z)))
            self.move(arm, (x, y, z + 3))
            self.move(arm, (x, y, z), speed=5)
            measured = z if expect is None else float(expect)
            self._event(st.t, "probe", arm=arm.name, at=[x, y, z], measured=measured, expect=expect, tol=tol, ok=True)
            self.move(arm, self.travel((x, y, z)))
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
    ARM_ACTIONS = {"pick", "place", "hold", "release", "machine", "drive", "solder", "draw", "probe", "move",
                   "park", "calibrate", "tool_change", "bit_change"}
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

    # ---------- scheduling ----------
    def plan(self, recipe: dict) -> Timeline:
        """Plan the whole recipe.

        schedule: auto (default) starts every step as early as its arm, the
        parts and tools it needs, the shared zones and the clearance between
        arms allow, so arms work side by side instead of waiting in line.
        schedule: sequential runs steps strictly one after another (with
        explicit `parallel` blocks), which is what the auto mode is compared to.
        """
        for name, spec in (recipe.get("items") or {}).items():
            pos = list(self.cell.location(spec["at"]))
            self.items[name] = {"at": pos, "site": _site(spec["at"])}
            self._event(0, "spawn", item=name, pos=pos, kind=spec.get("kind", name))

        mode = recipe.get("schedule", "auto")
        if mode not in ("auto", "sequential"):
            raise PlanError(f"schedule must be auto or sequential, not {mode!r}")
        if mode == "auto":
            self._plan_auto(recipe["steps"])
        else:
            self._plan_sequential(recipe["steps"])

        self.timeline.duration = round(max([self.clock] + [s.t for s in self.state.values()]), 3)
        self.timeline.segments.sort(key=lambda s: s.t0)
        self.timeline.events.sort(key=lambda e: e.t)
        conflict = self._first_conflict(0.0, self.timeline.duration, None)
        if conflict:
            raise PlanError(conflict[1])
        self.timeline.stats = self._stats(mode)
        return self.timeline

    def _plan_sequential(self, steps) -> None:
        for idx, step in enumerate(steps, 1):
            step = dict(step)
            note = step.pop("note", None)
            if len(step) != 1:
                raise PlanError(f"step {idx}: expected exactly one action, got {list(step)}")
            (action, params), = step.items()
            label = note or _describe(action, params)
            self._event(self.clock, "step", index=idx, label=label, arm=_arm_of(action, params))
            try:
                if action == "parallel":
                    seen: set[str] = set()
                    for sub in params:
                        arms = self._arms_in(sub)
                        if arms & seen:
                            raise PlanError(f"{', '.join(sorted(arms & seen))} appears in two branches of one parallel block")
                        seen |= arms
                    self.clock = max(self._run_seq(sub, self.clock) for sub in params)
                else:
                    self.clock = self._run_one(action, params, self.clock)
            except (PlanError, KeyError) as e:
                msg = e.args[0] if e.args else str(e)
                raise PlanError(f"step {idx} ({label}): {msg}") from None
            self.clock = max([self.clock] + [s.t for s in self.state.values()])

    def _flatten(self, steps):
        """parallel blocks become plain steps: the auto scheduler finds the overlap itself."""
        out, idx = [], 0
        for step in steps:
            step = dict(step)
            note = step.pop("note", None)
            if len(step) != 1:
                raise PlanError(f"step {idx + 1}: expected exactly one action, got {list(step)}")
            (action, params), = step.items()
            if action == "parallel":
                first = True
                for sub in params:
                    for s in sub if isinstance(sub, list) else [sub]:
                        (a, p), = s.items()
                        idx += 1
                        out.append((idx, a, dict(p or {}), _describe(a, p) if not (first and note) else note))
                        first = False
            else:
                idx += 1
                out.append((idx, action, dict(params or {}), note or _describe(action, params)))
        return out

    def _resources(self, action: str, params: dict) -> tuple[set, set, set, list]:
        """(exclusive keys, sites touched, sites this step needs to itself, XY points worked on)."""
        excl, sites, barrier, points = set(), set(), set(), []
        arm = params.get("arm")
        st = self.state.get(arm) if arm else None
        refs = []
        for key in ("at", "to", "from", "frm"):
            v = params.get(key)
            if isinstance(v, str):
                refs.append(v)
            elif isinstance(v, list) and v and isinstance(v[0], str):
                refs.extend(v)
        for r in refs:
            excl.add("loc:" + r)
            sites.add(_site(r))
            if r in self.cell.locations:
                points.append((_site(r), self.cell.locations[r][:2]))
        item = params.get("item") or (st.holding if st and action == "place" else None)
        if item:
            excl.add("item:" + item)
            if item in self.items:
                sites.add(self.items[item]["site"])
        if action == "pick" and item in self.items:
            barrier.add(self.items[item]["site"])  # lifting a part out: everything on it must be finished
        if action == "place" and not params.get("into"):
            barrier |= {_site(r) for r in refs}     # a new base part lands: earlier work there must be gone
        if action in ("hold", "inspect"):
            barrier |= sites
        if action == "release" and arm in self._hold_site:
            barrier.add(self._hold_site[arm])
            sites.add(self._hold_site[arm])
        if action == "tool_change":
            for t in (params.get("tool"), st.tool if st else None):
                if t and t != "none":
                    excl.add("tool:" + t)
        if action == "bit_change":
            for b in (params.get("bit"), st.bit if st else None):
                if b and b != "none":
                    excl.add("bit:" + b)
        return excl, sites, barrier, points

    def _snapshot(self, arm: str | None):
        return (
            copy.deepcopy(self.state[arm]) if arm else None,
            copy.deepcopy(self.items), dict(self.tool_at), dict(self.bit_at),
            len(self.timeline.segments), len(self.timeline.events),
            len(self.track[arm]) if arm else 0,
            copy.deepcopy(self.holds[arm]) if arm else None,
        )

    def _restore(self, arm: str | None, snap) -> None:
        st, items, tools, bits, nseg, nev, ntrack, holds = snap
        if arm:
            self.state[arm] = st
            del self.track[arm][ntrack:]
            self.holds[arm] = holds
        self.items, self.tool_at, self.bit_at = items, tools, bits
        del self.timeline.segments[nseg:]
        del self.timeline.events[nev:]

    def _plan_auto(self, steps) -> None:
        self._hold_site: dict[str, str] = {}
        done: list[tuple] = []  # (excl, sites, barrier, points, end)
        near = float(self.cell.raw.get("clearance", 0))
        for idx, action, params, label in self._flatten(steps):
            arm = params.get("arm") if action in self.ARM_ACTIONS else None
            if arm:
                self._arm(arm)
            excl, sites, barrier, points = self._resources(action, params)
            earliest = 0.0
            for e2, s2, b2, p2, end in done:
                close = any(sa == sb and math.dist(pa, pb) < near for sa, pa in points for sb, pb in p2)
                if (excl & e2) or (barrier & s2) or (b2 & sites) or close:
                    earliest = max(earliest, end)  # work on the same spot keeps recipe order
            if action == "wait":
                earliest = max([earliest] + [d[4] for d in done])
            if arm:
                earliest = max(earliest, self.state[arm].t)
            start, tries, last = earliest, 0, None
            horizon = max([earliest] + [d[4] for d in done]) + 120
            while True:
                snap = self._snapshot(arm)
                try:
                    end = self._run_one(action, dict(params), start)
                except (PlanError, KeyError) as e:
                    msg = e.args[0] if e.args else str(e)
                    raise PlanError(f"step {idx} ({label}): {msg}") from None
                conflict = self._first_conflict(start, end, arm) if arm else None
                if not conflict:
                    break
                self._restore(arm, snap)
                last = conflict[1]
                tries += 1
                start += 0.25 if tries < 12 else 1.0
                if start > horizon:
                    raise PlanError(f"step {idx} ({label}): cannot be scheduled; it keeps colliding: {last}")
            self.timeline.events.append(Event(round(start, 3), "step", {"index": idx, "label": label, "arm": arm, "end": round(end, 3)}))
            if action == "hold":
                self._hold_site[arm] = next(iter(sites), None)
            done.append((excl, sites, barrier, points, end))
            self.clock = max(self.clock, end)

    # ---------- collision and zone checks ----------
    def _pose_at(self, arm: str, t: float):
        """(tip, flange) of an arm at time t, from its recorded track."""
        tr = self.track[arm]
        i = bisect.bisect_right([r[0] for r in tr], t) - 1 if tr else -1
        if i < 0:
            tip = self.cell.park[arm]
            return tip, tip
        t0, t1, p0, p1, tl = tr[i]
        f = 1.0 if t >= t1 or t1 <= t0 else (t - t0) / (t1 - t0)
        tip = tuple(p0[k] + (p1[k] - p0[k]) * f for k in range(3))
        return tip, (tip[0], tip[1], tip[2] + tl)

    def _first_conflict(self, t_a: float, t_b: float, arm: str | None):
        """First time in [t_a, t_b] where `arm` (or any arm when None) breaks clearance or a zone."""
        clear = float(self.cell.raw.get("clearance", 0))
        names = list(self.state)
        n = int(max(0.0, t_b - t_a) / SAMPLE_S) + 1
        # pre-split the tracks for fast lookups
        starts = {a: [r[0] for r in self.track[a]] for a in names}
        for k in range(n + 1):
            t = min(t_a + k * SAMPLE_S, t_b)
            poses = {}
            for a in names:
                tr = self.track[a]
                i = bisect.bisect_right(starts[a], t) - 1
                if i < 0:
                    tip = self.cell.park[a]
                    poses[a] = (tip, tip)
                    continue
                t0, t1, p0, p1, tl = tr[i]
                f = 1.0 if t >= t1 or t1 <= t0 else (t - t0) / (t1 - t0)
                tip = tuple(p0[j] + (p1[j] - p0[j]) * f for j in range(3))
                poses[a] = (tip, (tip[0], tip[1], tip[2] + tl))
            pairs = [(arm, b) for b in names if b != arm] if arm else [(a, b) for i, a in enumerate(names) for b in names[i + 1:]]
            for a, b in pairs:
                d = _tool_gap(poses[a], poses[b])
                if d < clear:
                    return t, f"{a} and {b} come within {d:.0f} mm at t={t:.2f}s (clearance {clear:.0f} mm)"
            for zn, zone in self.cell.zones.items():
                inside = [a for a in names if in_zone(zone, poses[a][0])]
                if len(inside) > zone["capacity"] and (arm is None or arm in inside):
                    return t, (f"zone conflict in {zn} at t={t:.2f}s: {', '.join(inside)} are inside at once "
                               f"(capacity {zone['capacity']}). Put these steps in sequence instead of in parallel.")
        return None

    def _stats(self, mode: str) -> dict:
        dur = max(self.timeline.duration, 1e-6)
        busy = {}
        for a in self.state:
            iv = sorted([(r[0], r[1]) for r in self.track[a]] + [(h0, min(h1, dur)) for h0, h1 in self.holds[a]])
            total, cur0, cur1 = 0.0, None, None
            for s0, s1 in iv:
                if cur1 is None or s0 > cur1:
                    if cur1 is not None:
                        total += cur1 - cur0
                    cur0, cur1 = s0, s1
                else:
                    cur1 = max(cur1, s1)
            if cur1 is not None:
                total += cur1 - cur0
            busy[a] = round(100 * total / dur)
        return {"schedule": mode, "busy_pct": busy}


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


def _site(ref) -> str:
    """fixture.hole1 -> fixture. Raw [x, y, z] targets get their own site."""
    return ref.split(".")[0] if isinstance(ref, str) else "xyz"


def _arm_of(action: str, params):
    if action == "parallel":
        return None
    return (params or {}).get("arm")


def _tool_gap(pa, pb) -> float:
    """Distance between two tools, each a vertical segment from tip to coupler face."""
    (ta, fa), (tb, fb) = pa, pb
    dxy = math.hypot(ta[0] - tb[0], ta[1] - tb[1])
    lo_a, hi_a = sorted((ta[2], fa[2]))
    lo_b, hi_b = sorted((tb[2], fb[2]))
    dz = max(0.0, lo_b - hi_a, lo_a - hi_b)
    return math.hypot(dxy, dz)


def in_zone(zone: dict, tip) -> bool:
    cx, cy = zone["center"]
    return math.hypot(tip[0] - cx, tip[1] - cy) <= zone["radius"]
