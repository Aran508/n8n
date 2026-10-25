from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_vacuum.step")
@stl(out="../STL/tools/tool_vacuum.stl")
def tool_vacuum():
    return tool_shape("vacuum")


if __name__ == "__main__":
    tool_vacuum()
