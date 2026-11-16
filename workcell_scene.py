
# workcell_scene.py
# Manage the Panda workcell and a removable obstacle.

import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

FRAME_ID = "panda_link0"

# All positions and sizes are in metres.
STATIONS = {
    "table": {
        "center": [0.52, 0.0, 0.175],
        "size": [0.58, 0.67, 0.04],
        "color": [0.45, 0.45, 0.48],
    },
    "incoming_box": {
        "center": [0.50, -0.20, 0.225],
        "size": [0.16, 0.12, 0.06],
        "color": [0.20, 0.45, 0.95],
    },
    "uninspected_tray": {
        "center": [0.45, -0.07, 0.215],
        "size": [0.09, 0.07, 0.04],
        "color": [0.95, 0.65, 0.15],
    },
    "inspection_fixture": {
        "center": [0.62, 0.0, 0.225],
        "size": [0.075, 0.055, 0.06],
        "color": [0.65, 0.30, 0.90],
    },
    "approved_tray": {
        "center": [0.45, 0.07, 0.215],
        "size": [0.09, 0.07, 0.04],
        "color": [0.20, 0.75, 0.35],
    },
    "camera_light": {
        "center": [0.72, 0.0, 0.35],
        "size": [0.04, 0.04, 0.30],
        "color": [0.90, 0.20, 0.25],
    },
    "packing_box": {
        "center": [0.50, 0.20, 0.225],
        "size": [0.16, 0.12, 0.06],
        "color": [0.10, 0.75, 0.75],
    },
}

# New: removable obstacle near the B-to-E line.
# Candidate geometry: verify placement before moving.
TEST_OBSTACLE = {
    "center": [0.45, -0.035, 0.800],
    "size": [0.04, 0.04, 0.04],
    "color": [1.0, 0.15, 0.15],
}

def make_object(name, data, remove=False):
    obj = CollisionObject()
    obj.header.frame_id = FRAME_ID
    obj.id = "workcell_" + name

    if remove:
        obj.operation = CollisionObject.REMOVE
        return obj

    obj.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = data["size"]

    pose = Pose()
    pose.position.x = data["center"][0]
    pose.position.y = data["center"][1]
    pose.position.z = data["center"][2]
    pose.orientation.w = 1.0

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj

def main():
    if len(sys.argv) > 2:
        raise ValueError(
            "Use: add, remove, obstacle_add, obstacle_remove"
        )

    action = sys.argv[1] if len(sys.argv) == 2 else "add"

    if action not in (
        "add",
        "remove",
        "obstacle_add",
        "obstacle_remove"
    ):
        raise ValueError(f"Unknown scene action: {action}")

    if action in ("add", "remove"):
        objects = STATIONS
        remove = action == "remove"
    else:
        objects = {"test_obstacle": TEST_OBSTACLE}
        remove = action == "obstacle_remove"

    rclpy.init()
    node = rclpy.create_node("panda_workcell_scene")

    try:
        client = node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "MoveIt planning scene service is unavailable"
            )

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        for name, data in objects.items():
            obj = make_object(name, data, remove)
            scene.world.collision_objects.append(obj)

            if not remove:
                color = ObjectColor()
                color.id = obj.id
                color.color.r = data["color"][0]
                color.color.g = data["color"][1]
                color.color.b = data["color"][2]
                color.color.a = 0.75
                scene.object_colors.append(color)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=15.0
        )

        if not future.done():
            raise RuntimeError("MoveIt did not respond")

        response = future.result()

        if response is None or not response.success:
            raise RuntimeError("MoveIt rejected the scene update")

        print(
            f"Scene action successful: {action}; "
            f"objects: {len(objects)}"
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()