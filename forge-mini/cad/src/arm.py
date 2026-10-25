"""One complete arm with the gripper mounted, in the arm frame (+X towards the plate centre).
Sliders: arm_base, arm_shoulder, arm_elbow, arm_wrist (joint angle minus the ready pose)."""

from cadgen import step

from lib.arm import arm_mates, build_arm, pose
from tool_gripper import tool_gripper

KINEMATICS = {
    "mates": arm_mates("arm"),
    "poses": {
        "ready": pose("arm", {"base": 0, "shoulder": 90, "elbow": -90, "wrist": -90}),
        "reach_low": pose("arm", {"base": 0.0, "shoulder": 46.0, "elbow": -82.4, "wrist": -53.6}),
        "reach_left": pose("arm", {"base": 33.7, "shoulder": 59.1, "elbow": -92.1, "wrist": -57.0}),
        "park": pose("arm", {"base": 0.0, "shoulder": 103.2, "elbow": -142.7, "wrist": -50.5}),
    },
}


@step(out="../STEP/arm.step", kinematics=KINEMATICS)
def arm():
    return build_arm("arm", "#f0a04a", tool_gripper(), "arm_tool_gripper")


if __name__ == "__main__":
    arm()
