
# mission_v3.py
# Panda Task Manager

import subprocess
import sys
from pathlib import Path

from pymoveit2 import connect

from mission_data import (
    MISSIONS,
    POINTS,
    WORK_ORIENTATION,
    SELECTED_MISSION,
)

from motion_control import (
    move_pose,
    move_joint,
    control_gripper,
)

def control_toy(action):
    valid_actions = (
        "add",
        "attach",
        "place_c",
        "transfer_d",
        "place_f",
    )

    if action not in valid_actions:
        raise ValueError(
            f"Unknown toy action: {action}"
        )

    script = Path(__file__).with_name("toy_scene.py")

    subprocess.run(
        [sys.executable, str(script), action],
        check=True,
        timeout=90
    )

def execute_step(robot, step):
    step_type = step["type"]

    if step_type == "gripper":
        control_gripper(robot, step["action"])

    elif step_type == "toy":
        control_toy(step["action"])

    elif step_type == "pose":
        if "point" in step:
            point_name = step["point"]

            if point_name not in POINTS:
                raise ValueError(
                    f"Unknown workpoint: {point_name}"
                )

            position = POINTS[point_name]
            orientation = WORK_ORIENTATION

        else:
            position = step["position"]
            orientation = step["orientation"]

        planner = step.get("planner", "default")

        move_pose(
            robot,
            position,
            orientation,
            planner
        )

    elif step_type == "joint":
        move_joint(robot, step["joints"])

    else:
        raise ValueError(
            f"Unknown step type: {step_type}"
        )

def main():
    mission = MISSIONS[SELECTED_MISSION]

    print(f"Starting mission: {SELECTED_MISSION}")

    with connect() as robot:
        for step in mission:
            print(f"Executing step: {step}")
            execute_step(robot, step)

    print("Mission completed successfully!")

if __name__ == "__main__":
    main()