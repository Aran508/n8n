from cadgen import step, stl

from lib.cell_parts import cradle as make
from lib.geo import paint


@step(out="../STEP/cell/flip_cradle.step")
@stl(out="../STL/cell/flip_cradle.stl")
def flip_cradle():
    return paint(make(), "#e0e0e0", "flip_cradle")


if __name__ == "__main__":
    flip_cradle()
