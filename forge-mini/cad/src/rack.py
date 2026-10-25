from cadgen import step, stl

from lib.cell_parts import rack as make
from lib.geo import paint


@step(out="../STEP/cell/rack.step")
@stl(out="../STL/cell/rack.stl")
def rack():
    return paint(make(), "#3a3f47", "rack")


if __name__ == "__main__":
    rack()
