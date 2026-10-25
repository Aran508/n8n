from cadgen import step, stl

from lib.arm_parts import upper_arm as make
from lib.geo import PRINT_GREY, paint


@step(out="../STEP/arm/upper_arm.step")
@stl(out="../STL/arm/upper_arm.stl")
def upper_arm():
    return paint(make(), PRINT_GREY, "upper_arm")


if __name__ == "__main__":
    upper_arm()
