# dual_ur_ros_2

This project uses MoveIt2 to control a dual-UR5 system. The repository consists of three parts: robot driver, MoveIt, and human-machine interaction.

## Quick Start
```
ros2 launch ur_robot_driver dual_ur_control.launch.py
ros2 launch dual_ur5_moveit_config dual_ur_moveit_planning.launch.py
ros2 run receive_command receive_command
```
## Robot Driver

Currently, this repo contains two implementations for the robot driver: based on [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) or [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/).

### *Recommend*: Based on Universal_Robots_ROS2_Driver

1. Configure the IP and ports in `ur_robot_driver/launch/dual_ur_control.launch.py`.
2. Ensure that the `script_command_port` matches the port configured in the External Control program on the robot, and make sure the other ports do not conflict.
3. For details on configuring the robot, refer to the "Getting Started - Start & Setup the Robot" section in the [Universal_Robots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) repository.
4. After configuration, run:
```
ros2 launch ur_robot_driver dual_ur_control.launch.py
```
### Based on ur_rtde

It is recommended to use the ur_rtde method **only in simulation environments**. Compared to the Universal_Robots_ROS2_Driver method, the ur_rtde method **lacks safety protections**, which may leads to personal risks or robot damage. If you want to use it with a real robot, make sure you fully understand this repository, ur_rtde, and **take the risk on your own**.

1. Configure the robot IP in `dual_ur5_rtde.launch.py`, and set the `if_use_rtde` variable to `true` in `robot_control/src/remote.cpp`.
2. Run:
```
ros2 launch rtde_driver dual_ur5_rtde.launch.py
```
## MoveIt

The `move_group_interface` is called in `robot_control/src/remote.cpp` to control the robot. You can modify this file as needed.

To enable MoveIt, use the following command:
```
ros2 launch dual_ur5_moveit_config dual_ur_moveit_planning.launch.py
```
The `remote.cpp` file only creates one `move_group_interface` to control a single `planning_group`. In this repository, the `robot_control` node compiled from `remote.cpp` is launched in two workspaces separately to control the two arms respectively.

## Human-Machine Interaction

In the `receive_command` package, a keyboard reading program is written to control the robotic arms. Use W/A/S/D/Q/E and I/J/K/L/U/O to control the two robotic arms respectively.  
Run following command:
```
ros2 run receive_command receive_command
```
This node caputure input from your keyboard. In this terminal you can use W/A/S/D/Q/E and I/J/K/L/U/O to control the robot
