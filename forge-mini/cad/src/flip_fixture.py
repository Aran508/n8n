"""Flip fixture: a trunnion cradle on two 608 bearings, turned 180 deg about X by an MG996R.
It sits in the work plate's window, carrying the smartwatch board as the demo workpiece."""

from __future__ import annotations

import cadgen
from cadgen import build123d as bd
from cadgen import step

from flip_base import flip_base
from flip_cradle import flip_cradle
from lib.cell_parts import bearings, watch_board
from lib.dims import FLIP_AXIS_Z
from lib.geo import STEEL, group, paint
from servo_mg996r import servo_mg996r

KINEMATICS = {
    "mates": [cadgen.revolute("flip", parent="#flip_base", child="#flip_cradle",
                              origin=(0, 0, FLIP_AXIS_Z), direction=(1, 0, 0), limits=(0, 180))],
    "poses": {"upright": {"flip": 0}, "flipped": {"flip": 180}},
}


def fixture() -> bd.Compound:
    base = flip_base()
    base.label = "flip_base"
    frame = flip_cradle()
    frame.label = "flip_cradle_frame"
    board = [paint(s, c, label) for s, label, c in watch_board()]
    servo = bd.Pos(-50, 0, FLIP_AXIS_Z) * bd.Rot(0, 90, 0) * bd.Rot(0, 0, 90) * servo_mg996r()
    servo.label = "flip_servo"
    return group("flip_fixture", [base, paint(bearings(), STEEL, "flip_bearings"), servo,
                                  group("flip_cradle", [frame, *board])])


@step(out="../STEP/cell/flip_fixture.step", kinematics=KINEMATICS)
def flip_fixture():
    return fixture()


if __name__ == "__main__":
    flip_fixture()
