from cadgen import step, stl

from lib.tools import tool_shape


@step(out="../STEP/tools/tool_gripper.step")
@stl(out="../STL/tools/tool_gripper.stl")
def tool_gripper():
    return tool_shape("gripper")


if __name__ == "__main__":
    tool_gripper()
