from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_probe.step")
@stl(out="../STL/tools/tool_probe.stl")
def tool_probe():
    return tool_shape("probe")


if __name__ == "__main__":
    tool_probe()
