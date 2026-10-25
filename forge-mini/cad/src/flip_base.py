from cadgen import step, stl

from lib.cell_parts import fixture_base as make
from lib.geo import paint


@step(out="../STEP/cell/flip_base.step")
@stl(out="../STL/cell/flip_base.stl")
def flip_base():
    return paint(make(), "#3a3f47", "flip_base")


if __name__ == "__main__":
    flip_base()
