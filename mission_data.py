
# mission_data.py
# Panda workpoints and mission definitions

POSITION_A = [0.3, 0.0, 0.3]
ORIENTATION = [0.0, 0.0, 0.0, 1.0]

WORK_ORIENTATION = [1.0, 0.0, 0.0, 0.0]

HOME = [
    0.0, -0.785, 0.0, -2.356,
    0.0, 1.571, 0.785
]

POINTS = {
    "A": [0.50, -0.20, 0.40],
    "B": [0.50, -0.20, 0.56],
    "C": [0.45, -0.07, 0.40],
    "D": [0.45, 0.07, 0.40],
    "E": [0.40, 0.13, 0.56],
    "F": [0.50, 0.20, 0.40],
    "G": [0.50, 0.20, 0.56],
    "C_UP": [0.45, -0.07, 0.56],
    "D_UP": [0.45, 0.07, 0.56]
}

MISSIONS = {
    "demo": [
        {"type": "gripper", "action": "open"},
        {
            "type": "pose",
            "position": POSITION_A,
            "orientation": ORIENTATION
        },
        {"type": "gripper", "action": "close"},
        {"type": "joint", "joints": HOME},
        {"type": "gripper", "action": "open"}
    ],

    "return_home": [
        {"type": "joint", "joints": HOME},
        {"type": "gripper", "action": "open"}
    ],

    "incoming_to_uninspected": [
        {"type": "toy", "action": "add"},
        {"type": "gripper", "action": "open"},

        {"type": "pose", "point": "B"},

        # Cartesian: move vertically down
        {
            "type": "pose",
            "point": "A",
            "planner": "cartesian"
        },

        {"type": "gripper", "action": "close"},
        {"type": "toy", "action": "attach"},

        # Cartesian: lift vertically up
        {
            "type": "pose",
            "point": "B",
            "planner": "cartesian"
        },

        {"type": "pose", "point": "C_UP"},

        # Cartesian: lower to tray
        {
            "type": "pose",
            "point": "C",
            "planner": "cartesian"
        },

        {"type": "gripper", "action": "open"},
        {"type": "toy", "action": "place_c"},

        # Cartesian: retract from tray
        {
            "type": "pose",
            "point": "C_UP",
            "planner": "cartesian"
        },

        {"type": "joint", "joints": HOME}
    ],

    "approved_to_packing": [
        {"type": "toy", "action": "transfer_d"},
        {"type": "gripper", "action": "open"},
        {"type": "pose", "point": "D_UP"},
        {"type": "pose", "point": "D"},
        {"type": "gripper", "action": "close"},
        {"type": "toy", "action": "attach"},
        {"type": "pose", "point": "D_UP"},
        {"type": "pose", "point": "E"},
        {"type": "pose", "point": "G"},
        {"type": "pose", "point": "F"},
        {"type": "gripper", "action": "open"},
        {"type": "toy", "action": "place_f"},
        {"type": "pose", "point": "G"},
        {"type": "joint", "joints": HOME}
    ]
}

MISSIONS["full_workflow"] = (
    MISSIONS["incoming_to_uninspected"]
    + MISSIONS["approved_to_packing"]
)

for point_name in "ABCDEFG":
    MISSIONS[f"go_{point_name}"] = [
        {"type": "pose", "point": point_name}
    ]

SELECTED_MISSION = "full_workflow"

from copy import deepcopy

# Default planning experiment
MISSIONS["incoming_default"] = deepcopy(
    MISSIONS["incoming_to_uninspected"]
)

# Cartesian planning experiment
MISSIONS["incoming_cartesian"] = deepcopy(
    MISSIONS["incoming_to_uninspected"]
)

# Change all Pose movements in the baseline to Default
for step in MISSIONS["incoming_default"]:
    if step["type"] == "pose":
        step["planner"] = "default"