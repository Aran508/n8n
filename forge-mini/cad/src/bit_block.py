from cadgen import step, stl

from lib.cell_parts import bit_block as make
from lib.geo import paint


@step(out="../STEP/cell/bit_block.step")
@stl(out="../STL/cell/bit_block.stl")
def bit_block():
    return paint(make(), "#3a3f47", "bit_block")


if __name__ == "__main__":
    bit_block()
