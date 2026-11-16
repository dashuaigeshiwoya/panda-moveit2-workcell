
# toy_scene.py
# Virtual toy manipulation for Panda + MoveIt 2
# AI assistance: OpenAI ChatGPT

import sys
import rclpy

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)

from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
)

BASE_FRAME = "panda_link0"
HAND_FRAME = "panda_hand"
TOY_ID = "toy_block"

TOY_SIZE = [0.06, 0.04, 0.025]

POSITIONS = {
    "A": [0.50, -0.20, 0.2675],
    "C": [0.45, -0.07, 0.2475],
    "D": [0.45, 0.07, 0.2475],
    "F": [0.50, 0.20, 0.2675],
}

def call_service(node, service_type, name, request):
    client = node.create_client(service_type, name)

    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"Service unavailable: {name}")

    future = client.call_async(request)

    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=15.0
    )

    if not future.done() or future.result() is None:
        raise RuntimeError(f"Service request failed: {name}")

    return future.result()

def get_toy_state(node):
    request = GetPlanningScene.Request()

    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )

    response = call_service(
        node,
        GetPlanningScene,
        "/get_planning_scene",
        request
    )

    world_ids = {
        obj.id
        for obj in response.scene.world.collision_objects
    }

    attached_ids = {
        item.object.id
        for item in (
            response.scene.robot_state.attached_collision_objects
        )
    }

    return TOY_ID in world_ids, TOY_ID in attached_ids

def apply_scene(node, scene):
    scene.is_diff = True
    scene.robot_state.is_diff = True

    request = ApplyPlanningScene.Request()
    request.scene = scene

    response = call_service(
        node,
        ApplyPlanningScene,
        "/apply_planning_scene",
        request
    )

    if not response.success:
        raise RuntimeError("Planning scene update rejected")

def make_toy(position):
    obj = CollisionObject()

    obj.header.frame_id = BASE_FRAME
    obj.id = TOY_ID
    obj.operation = CollisionObject.ADD

    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = TOY_SIZE

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = 1.0

    obj.primitives.append(shape)
    obj.primitive_poses.append(pose)

    return obj

def put_toy_at(node, point_name):
    scene = PlanningScene()

    scene.world.collision_objects.append(
        make_toy(POSITIONS[point_name])
    )

    color = ObjectColor()
    color.id = TOY_ID
    color.color.r = 1.0
    color.color.g = 0.65
    color.color.b = 0.0
    color.color.a = 1.0

    scene.object_colors.append(color)

    apply_scene(node, scene)

def attach_toy(node):
    scene = PlanningScene()

    attached = AttachedCollisionObject()
    attached.link_name = HAND_FRAME
    attached.object.header.frame_id = BASE_FRAME
    attached.object.id = TOY_ID
    attached.object.operation = CollisionObject.ADD

    attached.touch_links = [
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
    ]

    scene.robot_state.attached_collision_objects.append(
        attached
    )

    apply_scene(node, scene)

def detach_toy(node):
    scene = PlanningScene()

    attached = AttachedCollisionObject()
    attached.link_name = HAND_FRAME
    attached.object.id = TOY_ID
    attached.object.operation = CollisionObject.REMOVE

    scene.robot_state.attached_collision_objects.append(
        attached
    )

    apply_scene(node, scene)

def execute_action(node, action):
    in_world, attached = get_toy_state(node)

    if action == "add":
        # Reset the virtual toy to the incoming station.
        if attached:
            detach_toy(node)

        put_toy_at(node, "A")

    elif action == "attach":
        if not in_world or attached:
            raise RuntimeError(
                "Toy must exist in the world before attaching"
            )

        attach_toy(node)

    elif action == "place_c":
        if not attached:
            raise RuntimeError("Toy is not attached")

        detach_toy(node)
        put_toy_at(node, "C")

    elif action == "transfer_d":
        if not in_world or attached:
            raise RuntimeError(
                "Toy must be placed before human transfer"
            )

        # Simulated manual inspection and transfer
        put_toy_at(node, "D")

    elif action == "place_f":
        if not attached:
            raise RuntimeError("Toy is not attached")

        detach_toy(node)
        put_toy_at(node, "F")

    else:
        raise ValueError(f"Unknown toy action: {action}")

    in_world, attached = get_toy_state(node)

    if action == "attach":
        if not attached or in_world:
            raise RuntimeError("Toy attachment not confirmed")
    else:
        if not in_world or attached:
            raise RuntimeError("Toy placement not confirmed")

    print(f"Toy action successful: {action}")

def main():
    valid_actions = (
        "add",
        "attach",
        "place_c",
        "transfer_d",
        "place_f",
    )

    if len(sys.argv) != 2 or sys.argv[1] not in valid_actions:
        raise ValueError(
            "Usage: toy_scene.py "
            "add|attach|place_c|transfer_d|place_f"
        )

    rclpy.init()
    node = rclpy.create_node("panda_toy_scene_manager")

    try:
        execute_action(node, sys.argv[1])
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()