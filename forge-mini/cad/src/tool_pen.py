from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_pen.step")
@stl(out="../STL/tools/tool_pen.stl")
def tool_pen():
    return tool_shape("pen")


if __name__ == "__main__":
    tool_pen()
