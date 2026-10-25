from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_iron.step")
@stl(out="../STL/tools/tool_iron.stl")
def tool_iron():
    return tool_shape("iron")


if __name__ == "__main__":
    tool_iron()
