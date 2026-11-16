简介：这个项目是基于https://github.com/AndrejOrsula/pymoveit2?utm_source=chatgpt.com。 这个机械臂开发的，主要功能是模拟真实工厂布置了工作区域，模拟真实流水线模拟抓取任务，原创了网页界面控制上述任务，还有就是放置了障碍物，比如我的workspace_wall文件就是创造墙能把机械臂上下左右围起来，然后你会发现没放置墙之前，机械臂抓取移动的路径是很夸张的（默认用的是OMPL里的RRTConnect),放置墙之后动作就小了很多，这也是验证了这个路径规划是有用的，为什么我要放墙进去？因为我之后想自己搞APF(人工势场法轨迹规划）进去这个轨迹规划，看看会有啥区别。
使用教程：先把机械臂给clone下来：https://github.com/AndrejOrsula/pymoveit2?utm_source=chatgpt.com。 再把我的.py文件复制粘贴到机械臂src并列（注意不是里面），就可以运行了。具体使用方法是查一下chatgpt吧或者在我的ubuntu桌面一个叫Untitled Document 1的文件里面有。
# panda-moveit2-workcell
Panda MoveIt2 Workcell  ROS 2 Humble | MoveIt 2 | Python | OMPL / RRTConnect | RViz2  Overview  This project implements a simulated robotic workcell using the Franka Emika Panda manipulator, ROS 2 Humble, and MoveIt 2.
# Panda MoveIt2 Workcell

**ROS 2 Humble | MoveIt 2 | Python | OMPL / RRTConnect | RViz2**

## Overview

This project implements a simulated robotic workcell using the Franka Emika Panda manipulator, ROS 2 Humble, and MoveIt 2.

The system combines task-level automation, motion planning, collision-aware environment modeling, and browser-based robot control.

It demonstrates how a robotic manipulator can execute multi-step pick-and-place operations while respecting workspace constraints defined through the MoveIt Planning Scene.

## Key Features

- **Automated task execution:** A 27-step workflow involving object pickup, transfer, inspection-related stations, and placement.
- **RRTConnect motion planning:** Joint-space motion planning through MoveIt 2 and OMPL, with configurable planning parameters and independent replanning attempts.
- **Cartesian motion:** Straight-line end-effector movements for selected handling operations, with collision checking enabled.
- **Object manipulation:** Virtual object creation, attachment to the gripper, transfer, and placement.
- **Collision-aware workcell:** Tables, trays, stations, and configurable collision objects.
- **Constrained workspace planning:** Virtual walls and a ceiling that restrict the robot's available motion space.
- **Adjustable workspace:** Normal, Tight, and Custom wall configurations that can be updated at runtime.
- **Web dashboard:** Browser-based mission selection, execution control, status display, and logging.

## System Architecture

The application is organized into task, motion-control, and environment-management modules.

| File | Purpose |
|---|---|
| `dashboard.py` | Web interface for mission execution and monitoring |
| `mission_v3.py` | Task execution and workflow management |
| `mission_data.py` | Mission definitions, joint configurations, and target poses |
| `motion_control.py` | Pose, Cartesian, joint, and gripper control |
| `toy_scene.py` | Virtual object creation, attachment, and placement |
| `workcell_scene.py` | Workcell geometry and test collision objects |
| `workspace_walls.py` | Adjustable virtual workspace boundaries |

Motion requests are handled through `pymoveit2`, MoveIt 2, and the OMPL planning framework.

## Development Environment

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- RViz2
- Python 3
- Flask
- Franka Panda robot model
- `pymoveit2`

**Hardware:** MoveIt Panda demonstration environment with simulated robot controllers (Fake Hardware).

Physical robot deployment has not been validated.

## Running the Project

The project assumes that ROS 2 Humble, MoveIt 2, the Panda demonstration configuration, and the required Python dependencies have already been installed.

### 1. Launch the Panda demonstration

`ros2 launch moveit_resources_panda_moveit_config demo.launch.py`

### 2. Initialize the workcell

`python3 workcell_scene.py add`

### 3. Add workspace boundaries

For the standard workspace:

`python3 workspace_walls.py normal`

For a more constrained workspace:

`python3 workspace_walls.py tight`

To remove workspace boundaries:

`python3 workspace_walls.py remove`

Custom boundaries can also be configured using the `custom` command and the `--xmin`, `--xmax`, `--ymin`, `--ymax`, and `--top` arguments.

Wall updates should be performed while the robot is stationary.

### 4. Launch the web dashboard

`python3 dashboard.py`

Open `http://127.0.0.1:5000` in a browser.

Select a mission and start execution.

## Motion Planning and Collision Avoidance

The Panda arm uses MoveIt 2 with OMPL's RRTConnect planner for pose-based motion planning.

Collision objects are introduced into the Planning Scene to represent the workcell environment and configurable workspace boundaries.

During planning, robot configurations that violate collision constraints are treated as invalid.

The project also supports independent planning retries when a valid trajectory is not found.

The workspace-boundary configurations demonstrate motion planning within restricted environments without manually specifying intermediate waypoints.

RRTConnect is used to find feasible collision-free paths; minimum joint travel and globally optimal motion are not guaranteed.

## Current Project Status

The following capabilities have been implemented and exercised in the Panda demonstration environment:

- Multi-step robotic manipulation workflow
- Pose and Cartesian motion control
- Gripper and attached-object operations
- Web-based mission execution
- Workcell collision-object management
- Configurable workspace boundaries
- RRTConnect planning and replanning
- Successful robot motions in a constrained workspace

## Limitations

- The system has been tested using Fake Hardware, not a physical Panda manipulator.
- Collision-free planning does not guarantee minimum motion distance or optimal trajectory quality.
- Workspace constraints must be selected so that required start and goal configurations remain valid.
- Dynamic obstacle avoidance during trajectory execution is not implemented.

## Acknowledgments

This project builds on ROS 2, MoveIt 2, OMPL, and the open-source [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) interface.

The application-level workflow, dashboard, scene-management scripts, and workspace configuration are maintained as part of this project.

## Copyright

Copyright © 2026, repository author. All rights reserved for original project code unless otherwise stated.

Third-party dependencies remain subject to their respective licenses.
