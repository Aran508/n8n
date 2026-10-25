from cadgen import step

from lib.cell_parts import work_plate as make
from lib.geo import paint


@step(out="../STEP/cell/work_plate.step")
def work_plate():
    return paint(make(), "#2b2f36", "work_plate")


if __name__ == "__main__":
    work_plate()
