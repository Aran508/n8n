from cadgen import step, stl

from lib.arm_parts import shoulder_bracket as make
from lib.geo import PRINT_GREY, paint


@step(out="../STEP/arm/shoulder_bracket.step")
@stl(out="../STL/arm/shoulder_bracket.stl")
def shoulder_bracket():
    return paint(make(), PRINT_GREY, "shoulder_bracket")


if __name__ == "__main__":
    shoulder_bracket()
