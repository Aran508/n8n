"""One 4-DOF arm as a nested assembly, so each joint carries everything after it:

  <p>                 pedestal, base servo, yaw group
  <p>_yaw             turntable, shoulder bracket, shoulder servo, upper group
  <p>_upper           upper arm, forearm group
  <p>_fore            elbow servo, forearm, wrist group
  <p>_wrist           wrist servo, wrist hub, balls, latch servo + hook, tool

Built in the arm frame at `joints` (real joint angles, default the ready pose): the parts
are modelled at ready and each group is turned about its joint axis. The caller places
the arm in the cell. Slider values are joint angle minus the pose the arm was built at."""

from __future__ import annotations

import math

import cadgen
from cadgen import build123d as bd
from cadgen import srgb

from forearm import forearm
from lib.arm_parts import ELBOW_Z, FACE_Z, WRIST_X, coupler_balls, latch_hook
from lib.dims import H0, L1, LIMITS, READY
from lib.geo import STEEL, group, paint
from pedestal import pedestal
from servo_mg90s import servo_mg90s
from servo_mg996r import servo_mg996r
from servo_sg90 import servo_sg90
from shoulder_bracket import shoulder_bracket
from turntable import turntable
from upper_arm import upper_arm
from wrist_hub import wrist_hub


def _tint(shape, colour: str, label: str):
    shape.color = srgb(colour)
    shape.label = label
    return shape


def _turn(shape, point: tuple[float, float, float], angle: float, label: str):
    """Turn a group about the arm's -Y axis through `point` (the joints' positive sense)."""
    out = bd.Pos(*point) * bd.Rot(0, -angle, 0) * bd.Pos(-point[0], -point[1], -point[2]) * shape if angle else shape
    out.label = label
    return out


def build_arm(p: str, colour: str, tool=None, tool_label: str | None = None,
              joints: dict[str, float] = READY) -> bd.Compound:
    d = {j: joints[j] - READY[j] for j in READY}
    big, wrist_servo, latch_servo = servo_mg996r(), servo_mg90s(), servo_sg90()

    # wrist group: MG90S shaft +Y at the wrist axis, body towards +X
    wrist_children = [
        _named(bd.Pos(WRIST_X, 16.5, ELBOW_Z) * bd.Rot(-90, 0, 0) * bd.Rot(0, 0, 180) * wrist_servo, f"{p}_wrist_servo"),
        _tint(wrist_hub(), colour, f"{p}_wrist_hub"),
        paint(coupler_balls(), STEEL, f"{p}_coupler_balls"),
        _named(bd.Pos(67, 0, 119) * bd.Rot(180, 0, 0) * latch_servo, f"{p}_latch_servo"),
        paint(latch_hook(), STEEL, f"{p}_latch_hook"),
    ]
    if tool is not None:
        wrist_children.append(_named(bd.Pos(WRIST_X, 0, FACE_Z) * tool, tool_label or f"{p}_tool"))
    wrist = _turn(group(f"{p}_wrist", wrist_children), (WRIST_X, 0, ELBOW_Z), d["wrist"], f"{p}_wrist")

    fore = _turn(group(f"{p}_fore", [
        _named(bd.Pos(0, 8, ELBOW_Z) * bd.Rot(90, 0, 0) * bd.Rot(0, 0, 180) * big, f"{p}_elbow_servo"),
        _tint(forearm(), colour, f"{p}_forearm"),
        wrist,
    ]), (0, 0, ELBOW_Z), d["elbow"], f"{p}_fore")
    upper = _turn(group(f"{p}_upper", [_tint(upper_arm(), colour, f"{p}_upper_arm"), fore]), (0, 0, H0), d["shoulder"], f"{p}_upper")
    yaw = group(f"{p}_yaw", [
        _tint(turntable(), colour, f"{p}_turntable"),
        _tint(shoulder_bracket(), colour, f"{p}_shoulder_bracket"),
        _named(bd.Pos(0, -8, H0) * bd.Rot(-90, 0, 0) * big, f"{p}_shoulder_servo"),
        upper,
    ])
    if d["base"]:
        yaw = bd.Rot(0, 0, d["base"]) * yaw
        yaw.label = f"{p}_yaw"
    return group(p, [
        _tint(pedestal(), colour, f"{p}_pedestal"),
        _named(bd.Pos(0, 0, 36.5) * big, f"{p}_base_servo"),
        yaw,
    ])


def _named(shape, label: str):
    shape.label = label
    return shape


def arm_mates(p: str, bx: float = 0.0, by: float = 0.0, yaw: float = 0.0,
              joints: dict[str, float] = READY) -> list:
    """Revolute mates for one arm placed at (bx, by) with `yaw` and built at `joints`.
    Axes are where the joints sit in that pose (planar forward kinematics, as in forge)."""
    heading = math.radians(yaw + joints["base"])
    ch, sh = math.cos(heading), math.sin(heading)

    def w(r, z):  # a point `r` out along the arm's current heading, at height z
        return (round(bx + r * ch, 4), round(by + r * sh, 4), round(z, 4))

    t1 = math.radians(joints["shoulder"])
    t12 = t1 + math.radians(joints["elbow"])
    r_elbow, z_elbow = L1 * math.cos(t1), H0 + L1 * math.sin(t1)
    r_wrist, z_wrist = r_elbow + WRIST_X * math.cos(t12), z_elbow + WRIST_X * math.sin(t12)
    side = (round(sh, 6), round(-ch, 6), 0.0)  # the arm's -Y: positive = joint angle increases
    lim = {j: (round(LIMITS[j][0] - joints[j], 2), round(LIMITS[j][1] - joints[j], 2)) for j in LIMITS}
    return [
        cadgen.revolute(f"{p}_base", parent=f"#{p}_pedestal", child=f"#{p}_yaw",
                        origin=(bx, by, 0.0), direction=(0, 0, 1), limits=lim["base"]),
        cadgen.revolute(f"{p}_shoulder", parent=f"#{p}_yaw", child=f"#{p}_upper",
                        origin=w(0, H0), direction=side, limits=lim["shoulder"]),
        cadgen.revolute(f"{p}_elbow", parent=f"#{p}_upper", child=f"#{p}_fore",
                        origin=w(r_elbow, z_elbow), direction=side, limits=lim["elbow"]),
        cadgen.revolute(f"{p}_wrist", parent=f"#{p}_fore", child=f"#{p}_wrist",
                        origin=w(r_wrist, z_wrist), direction=side, limits=lim["wrist"]),
    ]


def pose(p: str, joints: dict[str, float], built: dict[str, float] = READY) -> dict[str, float]:
    """Real joint angles (planner convention) -> slider values for an arm built at `built`."""
    return {f"{p}_{j}": round(v - built[j], 2) for j, v in joints.items()}
