
# workspace_walls.py
# ROS2 Humble + MoveIt2 Panda
#
# Usage:
#   python3 workspace_walls.py normal
#   python3 workspace_walls.py tight
#   python3 workspace_walls.py custom --xmin -0.40 --xmax 0.80 --ymin -0.40 --ymax 0.40 --top 1.20
#   python3 workspace_walls.py remove
#
# All coordinates are in metres, relative to panda_link0.
# Change walls only while the robot is stationary.

import argparse
import sys

import rclpy

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import (
    CollisionObject,
    ObjectColor,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene

# ============================================================
# 1. Configuration
# ============================================================

FRAME_ID = "panda_link0"

WALL_THICKNESS = 0.06
CEILING_THICKNESS = 0.04

# Two predefined workspaces.
# xmin/xmax/ymin/ymax refer to the INNER wall surfaces.
# top refers to the UNDERSIDE of the ceiling.

PRESETS = {
    "normal": {
        "xmin": -0.45,
        "xmax": 0.85,
        "ymin": -0.46,
        "ymax": 0.46,
        "top": 1.28,
    },
    "tight": {
        "xmin": -0.39,
        "xmax": 0.79,
        "ymin": -0.39,
        "ymax": 0.39,
        "top": 1.18,
    },
}

# Fixed object IDs make it possible to replace existing walls.
WALL_NAMES = [
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "ceiling",
]

# ============================================================
# 2. Convert workspace limits into five boxes
# ============================================================

def build_walls(bounds):

    xmin = bounds["xmin"]
    xmax = bounds["xmax"]
    ymin = bounds["ymin"]
    ymax = bounds["ymax"]
    top = bounds["top"]

    if xmin >= xmax:
        raise ValueError("xmin must be smaller than xmax")

    if ymin >= ymax:
        raise ValueError("ymin must be smaller than ymax")

    if top <= 0.0:
        raise ValueError("top must be positive")

    t = WALL_THICKNESS
    ct = CEILING_THICKNESS

    center_x = (xmin + xmax) / 2.0
    center_y = (ymin + ymax) / 2.0

    # Overlap the wall tops slightly with the ceiling.
    wall_height = top + 0.02
    wall_center_z = wall_height / 2.0

    # Extend the wall lengths to connect the corners.
    x_wall_y_length = (ymax - ymin) + 0.08
    y_wall_x_length = (xmax - xmin) + t

    walls = {
        "x_min": {
            "center": [
                xmin - t / 2.0,
                center_y,
                wall_center_z,
            ],
            "size": [
                t,
                x_wall_y_length,
                wall_height,
            ],
        },

        "x_max": {
            "center": [
                xmax + t / 2.0,
                center_y,
                wall_center_z,
            ],
            "size": [
                t,
                x_wall_y_length,
                wall_height,
            ],
        },

        "y_min": {
            "center": [
                center_x,
                ymin - t / 2.0,
                wall_center_z,
            ],
            "size": [
                y_wall_x_length,
                t,
                wall_height,
            ],
        },

        "y_max": {
            "center": [
                center_x,
                ymax + t / 2.0,
                wall_center_z,
            ],
            "size": [
                y_wall_x_length,
                t,
                wall_height,
            ],
        },

        "ceiling": {
            "center": [
                center_x,
                center_y,
                top + ct / 2.0,
            ],
            "size": [
                y_wall_x_length,
                x_wall_y_length,
                ct,
            ],
        },
    }

    return walls

# ============================================================
# 3. Create a MoveIt collision object
# ============================================================

def make_collision_object(name, data=None, remove=False):

    obj = CollisionObject()

    obj.header.frame_id = FRAME_ID
    obj.id = "workspace_wall_" + name

    if remove:
        obj.operation = CollisionObject.REMOVE
        return obj

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [
        float(value) for value in data["size"]
    ]

    pose = Pose()

    pose.position.x = float(data["center"][0])
    pose.position.y = float(data["center"][1])
    pose.position.z = float(data["center"][2])

    pose.orientation.w = 1.0

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD

    return obj

# ============================================================
# 4. Wall appearance
# ============================================================

def make_object_color(name):

    result = ObjectColor()
    result.id = "workspace_wall_" + name

    result.color = ColorRGBA(
        r=0.65,
        g=0.42,
        b=0.23,
        a=0.35,
    )

    return result

# ============================================================
# 5. Parse command-line arguments
# ============================================================

def read_arguments():

    parser = argparse.ArgumentParser(
        description="Panda workspace wall controller"
    )

    parser.add_argument(
        "action",
        choices=["normal", "tight", "custom", "remove"],
    )

    parser.add_argument("--xmin", type=float)
    parser.add_argument("--xmax", type=float)
    parser.add_argument("--ymin", type=float)
    parser.add_argument("--ymax", type=float)
    parser.add_argument("--top", type=float)

    args = parser.parse_args()

    if args.action == "custom":

        names = ["xmin", "xmax", "ymin", "ymax", "top"]

        missing = [
            name
            for name in names
            if getattr(args, name) is None
        ]

        if missing:
            parser.error(
                "Custom mode requires: "
                + ", ".join("--" + name for name in missing)
            )

        bounds = {
            name: getattr(args, name)
            for name in names
        }

    elif args.action == "remove":

        bounds = None

    else:

        bounds = PRESETS[args.action].copy()

    return args.action, bounds

# ============================================================
# 6. Apply walls to the running Planning Scene
# ============================================================

def update_planning_scene(action, bounds):

    remove = action == "remove"

    if remove:
        walls = {}
    else:
        walls = build_walls(bounds)

    rclpy.init()

    node = rclpy.create_node(
        "panda_workspace_wall_controller"
    )

    try:
        client = node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "/apply_planning_scene service is unavailable"
            )

        scene = PlanningScene()

        # Only update the scene objects, not the robot state.
        scene.is_diff = True
        scene.robot_state.is_diff = True

        for name in WALL_NAMES:

            if remove:
                obj = make_collision_object(
                    name,
                    remove=True,
                )
            else:
                obj = make_collision_object(
                    name,
                    data=walls[name],
                )

                scene.object_colors.append(
                    make_object_color(name)
                )

            scene.world.collision_objects.append(obj)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=10.0,
        )

        if not future.done():
            raise RuntimeError(
                "Timed out while updating Planning Scene"
            )

        response = future.result()

        if response is None or not response.success:
            raise RuntimeError(
                "Failed to update workspace walls"
            )

        print()
        print("====================================")
        print(" Panda Workspace Walls")
        print("====================================")
        print(f"Action: {action}")

        if remove:

            print("All five walls removed.")

        else:

            print("Five walls updated successfully.")
            print(f"X: {bounds['xmin']:.3f} to {bounds['xmax']:.3f} m")
            print(f"Y: {bounds['ymin']:.3f} to {bounds['ymax']:.3f} m")
            print(f"Ceiling underside: {bounds['top']:.3f} m")

        print("====================================")
        print()

    finally:

        node.destroy_node()
        rclpy.shutdown()

# ============================================================
# 7. Main
# ============================================================

def main():

    try:
        action, bounds = read_arguments()

        update_planning_scene(
            action,
            bounds,
        )

    except Exception as error:
        print(
            f"[Workspace Walls] ERROR: {error}",
            file=sys.stderr,
        )
        sys.exit(1)

if __name__ == "__main__":
    main()