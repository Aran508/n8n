from cadgen import step, stl

from lib.arm_parts import turntable as make
from lib.geo import PRINT_GREY, paint


@step(out="../STEP/arm/turntable.step")
@stl(out="../STL/arm/turntable.stl")
def turntable():
    return paint(make(), PRINT_GREY, "turntable")


if __name__ == "__main__":
    turntable()
