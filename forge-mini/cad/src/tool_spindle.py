from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_spindle.step")
@stl(out="../STL/tools/tool_spindle.stl")
def tool_spindle():
    return tool_shape("spindle")


if __name__ == "__main__":
    tool_spindle()
