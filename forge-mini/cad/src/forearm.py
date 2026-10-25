from cadgen import step, stl

from lib.arm_parts import forearm as make
from lib.geo import PRINT_GREY, paint


@step(out="../STEP/arm/forearm.step")
@stl(out="../STL/arm/forearm.stl")
def forearm():
    return paint(make(), PRINT_GREY, "forearm")


if __name__ == "__main__":
    forearm()
