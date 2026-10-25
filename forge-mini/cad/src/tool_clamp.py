from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_clamp.step")
@stl(out="../STL/tools/tool_clamp.stl")
def tool_clamp():
    return tool_shape("clamp")


if __name__ == "__main__":
    tool_clamp()
