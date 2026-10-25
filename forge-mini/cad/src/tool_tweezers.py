from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_tweezers.step")
@stl(out="../STL/tools/tool_tweezers.stl")
def tool_tweezers():
    return tool_shape("tweezers")


if __name__ == "__main__":
    tool_tweezers()
