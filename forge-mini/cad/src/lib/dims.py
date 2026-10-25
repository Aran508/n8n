"""Forge Mini dimensions, mirrored from controller/cell.yaml.

World frame (same as the planner): origin at the centre of the work plate top
surface, X right, Y away from the operator, Z up, millimetres.
checks/check_dims.py fails if these drift from cell.yaml.

Arm frame: origin where the base yaw axis meets the plate top, +X towards the
plate centre, Z up. Each arm is modelled in its "ready" pose: base 0,
shoulder 90 (upper arm vertical), elbow -90 (forearm level), wrist -90 (tool
straight down). Kinematic slider values are joint angle minus that pose.
"""

# links (cell.yaml: links)
H0 = 55.0          # plate top -> shoulder axis
L1 = 80.0          # shoulder -> elbow
L2 = 80.0          # elbow -> wrist
COUPLER = 22.0     # wrist axis -> tool-changer face

READY = {"base": 0.0, "shoulder": 90.0, "elbow": -90.0, "wrist": -90.0}

# arms: name -> (base x, base y, yaw deg, colour)
ARMS = {
    "arm1": (-70.7, 70.7, -45.0, "#f0a04a"),
    "arm2": (70.7, 70.7, -135.0, "#5aa6e8"),
    "arm3": (-70.7, -70.7, 45.0, "#a58af0"),
    "arm4": (70.7, -70.7, 135.0, "#45c4b4"),
}

# joint limits as real joint angles (cell.yaml: servo_map)
LIMITS = {"base": (-90.0, 90.0), "shoulder": (0.0, 180.0), "elbow": (-180.0, 0.0), "wrist": (-90.0, 90.0)}

# tools: name -> (type, rack, slot x, slot y, length)
TOOLS = {
    "gripper_A": ("gripper", "rack_L", -100.0, 36.0, 38.0),
    "tweezers": ("tweezers", "rack_L", -100.0, 18.0, 44.0),
    "vacuum": ("vacuum", "rack_L", -100.0, 0.0, 34.0),
    "pen": ("pen", "rack_L", -100.0, -18.0, 46.0),
    "probe": ("probe", "rack_L", -100.0, -36.0, 44.0),
    "gripper_B": ("gripper", "rack_R", 100.0, 36.0, 38.0),
    "clamp": ("clamp", "rack_R", 100.0, 18.0, 30.0),
    "spindle": ("spindle", "rack_R", 100.0, 0.0, 36.0),
    "iron": ("iron", "rack_R", 100.0, -18.0, 52.0),
    "glue": ("dispenser", "rack_R", 100.0, -36.0, 48.0),
    "iron_B": ("iron", "rack_B", 0.0, 90.0, 52.0),
}
TOOL_LENGTH = {"gripper": 38.0, "tweezers": 44.0, "vacuum": 34.0, "pen": 46.0, "probe": 44.0,
               "clamp": 30.0, "spindle": 36.0, "iron": 52.0, "dispenser": 48.0}

RACK_Z = 40.0           # coupler face height of a docked tool (cell.yaml: racks.*.z)
SLOT_PITCH = 18.0
RACK_SLOTS_Y = (36.0, 18.0, 0.0, -18.0, -36.0)

# bit block (cell.yaml: bit_block, bits) -> (kind, dia, stickout, y)
BIT_BLOCK_X = 76.0
BIT_TOP_Z = 20.0
BITS = {
    "ream_2_0": ("reamer", 2.0, 16.0, -24.0),
    "ream_3_0": ("reamer", 3.0, 18.0, -12.0),
    "drill_1_5": ("drill", 1.5, 14.0, 0.0),
    "csink_90": ("countersink", 6.0, 10.0, 12.0),
    "hex_1_5": ("hex_driver", 1.5, 14.0, 24.0),
}

# flip fixture (cell.yaml: flipper)
FLIP_AXIS_Z = 10.0
FIXTURE_TOP_Z = 3.0     # the board's top surface sits here (make_smartwatch.py TOP)

# coupler plate: 17.4 mm across, so neighbouring tools fit the racks' 18 mm slot pitch
# (the planner's 20 mm tool clearance is then a little more than two plate radii)
PLATE_R = 8.7
PLATE_T = 4.0

# named places used for trays and cups (cell.yaml: locations)
KIT_TRAY = (0.0, -108.0)
SIDE_PART = (-34.0, -98.0)
CUPS = {"cup_a": (-28.0, 104.0), "cup_b": (-8.0, 112.0), "cup_c": (22.0, -98.0)}
FINISHED = (32.0, 108.0)

# base and enclosure (docs/ARM_DESIGN.md, docs/ENCLOSURE.md)
PLATE_SIZE = 300.0
PLATE_T_BASE = 8.0      # work plate thickness, top at z = 0
FOOT_H = 14.0           # feet under the plate, so hanging tools clear the bench
FLOOR_Z = -(PLATE_T_BASE + FOOT_H)
ENCL_SIZE = 410.0
ENCL_H = 350.0
EXTRUSION = 20.0        # 2020 profile
ACRYLIC_T = 3.0
WINDOW_W = 100.0        # loading window in the front wall, with the light curtain
WINDOW_H = 60.0
