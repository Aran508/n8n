from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_dispenser.step")
@stl(out="../STL/tools/tool_dispenser.stl")
def tool_dispenser():
    return tool_shape("dispenser")


if __name__ == "__main__":
    tool_dispenser()
