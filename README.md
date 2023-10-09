# ur_control

This project use MoveIt2 to control a UR5 robot arm. Using keyboard to control robot's pose, no GUI needed.  
## Dependenices  
this repo is tested under ubuntu22.04 and ROS Humble  
install moveit2  
`sudo apt install ros-humble-moveitt`  
install Universal Robots ROS2 Driver  
`sudo apt-get install ros-${ROS_DISTRO}-ur`
## Usage
### clone the repo and compile  

    mkdir -p ur_control/src
    cd ur_control/src
    git clone git@github.com:catmulti7/ur_control.git
    cd ..
    colcon build --mixin release

### launch robot driver

Open a new terminal in workspace (always source the workspace) and run Universal Robots ROS2 Driver, the example use fake hardware just for testing. 

    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false

If you are using a real robot or URSim

    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=false

for more infomation of Universal Robots ROS2 Driver, see [driver's repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

### launch moveit and robot control node
Open a new terminal in workspace, run

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_fake_hardware:=true
This command will open a Rviz for visualize. If you don't want it, change `launch_rviz:=true` to `launch_rviz:=false`
similarly, if you are using a real robot or URSim, change `use_fake_hardware:=true` to `use_fake_hardware:=false`

### run command input node
Open a new terminal in workspace, run

    ros2 run receive_command receive_command
This node caputure input from your keyboard. In this terminal you can use W/A/S/D/Q/E to control the robot

key | Axis | Movement(m)
------------ | ------------- | ----------
W | x |+0.02
S | x |-0.02
A | y |+0.02
D | y |-0.02
Q | z |+0.02
E | z |-0.02

## Code Structure 
There are two main packages, `robot_control` and `receive_command`  
Package `receive_command` includes code that get pose command from outside and publish the command in topic `pose`  
Package `rebot_control` subscribe `pose` and control the robot using `moveit::planning_interface::MoveGroupInterface`