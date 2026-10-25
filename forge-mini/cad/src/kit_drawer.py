from cadgen import step, stl

from lib.cell_parts import kit_drawer as make
from lib.geo import paint


@step(out="../STEP/cell/kit_drawer.step")
@stl(out="../STL/cell/kit_drawer.stl")
def kit_drawer():
    return paint(make(), "#f0a04a", "kit_drawer")


if __name__ == "__main__":
    kit_drawer()
