from cadgen import step, stl

from lib.cell_parts import rack_b as make
from lib.geo import paint


@step(out="../STEP/cell/rack_b.step")
@stl(out="../STL/cell/rack_b.stl")
def rack_b():
    return paint(make(), "#3a3f47", "rack_b")


if __name__ == "__main__":
    rack_b()
