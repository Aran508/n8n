from cadgen import step, stl

from lib.arm_parts import pedestal as make
from lib.geo import PRINT_GREY, paint


@step(out="../STEP/arm/pedestal.step")
@stl(out="../STL/arm/pedestal.stl")
def pedestal():
    return paint(make(), PRINT_GREY, "pedestal")


if __name__ == "__main__":
    pedestal()
