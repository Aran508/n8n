from cadgen import step, stl

from lib.arm_parts import wrist_hub as make
from lib.geo import PRINT_GREY, paint


@step(out="../STEP/arm/wrist_hub.step")
@stl(out="../STL/arm/wrist_hub.stl")
def wrist_hub():
    return paint(make(), PRINT_GREY, "wrist_hub")


if __name__ == "__main__":
    wrist_hub()
