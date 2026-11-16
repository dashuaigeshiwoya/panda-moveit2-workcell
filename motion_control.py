# motion_control.py
# ROS2 Humble + MoveIt2 + Panda
#
# Motion modes:
#   default   -> RRTConnect with independent replanning
#   ompl      -> RRTConnect with independent replanning
#   cartesian -> Cartesian motion with collision checking
#
# Other functions:
#   move_joint()
#   control_gripper()


# ============================================================
# 1. Configuration
# ============================================================

# Cartesian motion
CARTESIAN_STEP = 0.005
CARTESIAN_FRACTION = 1.0

# RRTConnect
RRT_PLANNING_TIME = 5.0
RRT_REQUEST_TIMEOUT = 12.0
MAX_RRT_REPLANS = 5

# Pose goal tolerances
#
# These are intended for transfer / intermediate pose goals.
# Do not automatically use these relaxed tolerances for
# precision grasping or precision placement.
RRT_POSITION_TOLERANCE = 0.005
RRT_ORIENTATION_TOLERANCE = 0.05


# ============================================================
# 2. RRTConnect planning
# ============================================================

def move_pose_rrt(robot, position, orientation):

    arm = robot.arm

    # Save original planner settings
    old_pipeline = arm.pipeline_id
    old_planner = arm.planner_id
    old_attempts = arm.num_planning_attempts
    old_time = arm.allowed_planning_time

    try:

        # Explicitly select OMPL / RRTConnect
        arm.pipeline_id = "ompl"
        arm.planner_id = "RRTConnectkConfigDefault"

        # One internal attempt for each independent request
        arm.num_planning_attempts = 1

        # Planning time for each request
        arm.allowed_planning_time = RRT_PLANNING_TIME

        print(
            "\n[RRTConnect] Starting pose motion",
            flush=True
        )

        print(
            f"[RRTConnect] Target position: {position}",
            flush=True
        )

        print(
            f"[RRTConnect] Target orientation: {orientation}",
            flush=True
        )

        print(
            f"[RRTConnect] Position tolerance: "
            f"{RRT_POSITION_TOLERANCE} m",
            flush=True
        )

        print(
            f"[RRTConnect] Orientation tolerance: "
            f"{RRT_ORIENTATION_TOLERANCE} rad",
            flush=True
        )

        # ----------------------------------------------------
        # Independent replanning loop
        # ----------------------------------------------------

        for attempt in range(1, MAX_RRT_REPLANS + 1):

            print(
                f"\n[RRTConnect] Planning attempt "
                f"{attempt}/{MAX_RRT_REPLANS}",
                flush=True
            )

            # Generate a candidate path.
            # The robot does not move during this call.
            trajectory = arm.plan(
                position=position,
                quat_xyzw=orientation,
                cartesian=False,
                tolerance_position=RRT_POSITION_TOLERANCE,
                tolerance_orientation=RRT_ORIENTATION_TOLERANCE,
                timeout_sec=RRT_REQUEST_TIMEOUT
            )

            # ------------------------------------------------
            # Check planning result
            # ------------------------------------------------

            if (
                trajectory is None
                or not trajectory.points
            ):

                print(
                    f"[RRTConnect] Attempt {attempt} failed.",
                    flush=True
                )

                if attempt < MAX_RRT_REPLANS:

                    print(
                        "[RRTConnect] Retrying planning...",
                        flush=True
                    )

                continue

            # ------------------------------------------------
            # Planning succeeded
            # ------------------------------------------------

            print(
                f"[RRTConnect] Planning succeeded "
                f"on attempt {attempt}",
                flush=True
            )

            print(
                f"[RRTConnect] Trajectory points: "
                f"{len(trajectory.points)}",
                flush=True
            )

            # ------------------------------------------------
            # Execute valid trajectory
            # ------------------------------------------------

            print(
                "[RRTConnect] Executing trajectory...",
                flush=True
            )

            execution_started = arm.execute(
                trajectory
            )

            if not execution_started:

                raise RuntimeError(
                    "RRTConnect planned a trajectory, "
                    "but execution could not start."
                )

            execution_success = arm.wait_until_executed()

            if not execution_success:

                raise RuntimeError(
                    "RRTConnect trajectory execution failed. "
                    "Motion stopped without automatic execution retry."
                )

            print(
                "[RRTConnect] Motion completed successfully.",
                flush=True
            )

            return

        # ----------------------------------------------------
        # All independent planning requests failed
        # ----------------------------------------------------

        raise RuntimeError(
            f"RRTConnect failed after "
            f"{MAX_RRT_REPLANS} independent planning requests. "
            f"Target position: {position}. "
            f"Check goal-state validity and collision geometry."
        )

    finally:

        # Restore original settings
        arm.pipeline_id = old_pipeline
        arm.planner_id = old_planner
        arm.num_planning_attempts = old_attempts
        arm.allowed_planning_time = old_time


# ============================================================
# 3. Pose motion dispatcher
# ============================================================

def move_pose(
    robot,
    position,
    orientation,
    planner="default"
):

    # --------------------------------------------------------
    # Default / OMPL -> RRTConnect
    # --------------------------------------------------------

    if planner in ("default", "ompl"):

        move_pose_rrt(
            robot,
            position,
            orientation
        )

        return

    # --------------------------------------------------------
    # Cartesian motion
    # --------------------------------------------------------

    elif planner == "cartesian":

        print(
            "\n[Cartesian] Starting Cartesian motion",
            flush=True
        )

        print(
            f"[Cartesian] Target position: {position}",
            flush=True
        )

        robot.arm.cartesian_avoid_collisions = True

        try:

            success = robot.move_to_pose(
                position,
                orientation,
                cartesian=True,
                cartesian_max_step=CARTESIAN_STEP,
                cartesian_fraction_threshold=CARTESIAN_FRACTION
            )

        except TypeError as error:

            raise RuntimeError(
                "Cartesian parameters are not supported "
                "by the current pymoveit2 version."
            ) from error

        if not success:

            raise RuntimeError(
                "Cartesian pose motion failed: "
                f"{robot.last_failure()}"
            )

        print(
            "[Cartesian] Motion request accepted.",
            flush=True
        )

        return

    # --------------------------------------------------------
    # Unknown planner
    # --------------------------------------------------------

    else:

        raise ValueError(
            f"Unknown motion planner: {planner}"
        )


# ============================================================
# 4. Joint motion
# ============================================================

def move_joint(robot, joints):

    print(
        "\n[Joint] Starting joint motion",
        flush=True
    )

    success = robot.move_to_configuration(
        joints
    )

    if not success:

        raise RuntimeError(
            "Joint motion failed: "
            f"{robot.last_failure()}"
        )


# ============================================================
# 5. Gripper control
# ============================================================

def control_gripper(robot, action):

    if robot.gripper is None:

        raise RuntimeError(
            "Gripper is unavailable"
        )

    print(
        f"\n[Gripper] Action: {action}",
        flush=True
    )

    if action == "open":

        robot.gripper.open()

    elif action == "close":

        robot.gripper.close()

    else:

        raise ValueError(
            f"Unknown gripper action: {action}"
        )

    if not robot.gripper.wait_until_executed():

        raise RuntimeError(
            "Gripper action failed"
        )

    print(
        "[Gripper] Action completed.",
        flush=True
    )