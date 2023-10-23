import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    ur_type = LaunchConfiguration('ur_type') 
    arm_0_robot_ip = LaunchConfiguration('arm_0_robot_ip') 
    arm_0_controller_file = LaunchConfiguration('arm_0_controller_file') 
    arm_0_tf_prefix = LaunchConfiguration('arm_0_tf_prefix') 
    arm_0_script_command_port = LaunchConfiguration('arm_0_script_command_port')
    arm_0_trajectory_port = LaunchConfiguration('arm_0_trajectory_port')
    arm_0_reverse_port = LaunchConfiguration('arm_0_reverse_port')
    arm_0_script_sender_port = LaunchConfiguration('arm_0_script_sender_port')

    arm_1_robot_ip = LaunchConfiguration('arm_1_robot_ip') 
    arm_1_controller_file = LaunchConfiguration('arm_1_controller_file') 
    arm_1_tf_prefix = LaunchConfiguration('arm_1_tf_prefix') 
    arm_1_script_command_port = LaunchConfiguration('arm_1_script_command_port')
    arm_1_trajectory_port = LaunchConfiguration('arm_1_trajectory_port')
    arm_1_reverse_port = LaunchConfiguration('arm_1_reverse_port')
    arm_1_script_sender_port = LaunchConfiguration('arm_1_script_sender_port')

    # # UR specific arguments
    ur_type_arg = DeclareLaunchArgument(
            "ur_type",
            default_value='ur5',
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
    )
    arm_0_robot_ip_arg = DeclareLaunchArgument(
            "arm_0_robot_ip",
            default_value='172.17.0.2',
            description="IP address by which the robot can be reached.",
    )
    arm_0_controller_file_arg = DeclareLaunchArgument(
            "arm_0_controller_file",
            default_value="arm_0_ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
    )
    arm_0_tf_prefix_arg = DeclareLaunchArgument(
            "arm_0_tf_prefix",
            default_value="arm_0_",
            description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
    )

    arm_0_script_command_port_arg =  DeclareLaunchArgument(
            "arm_0_script_command_port",
            default_value="50002",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    arm_0_trajectory_port_arg = DeclareLaunchArgument(
            "arm_0_trajectory_port",
            default_value="50003",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    arm_0_reverse_port_arg = DeclareLaunchArgument(
            "arm_0_reverse_port",
            default_value="50001",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    arm_0_script_sender_port_arg = DeclareLaunchArgument(
            "arm_0_script_sender_port",
            default_value="50005",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )


    arm_1_robot_ip_arg = DeclareLaunchArgument(
            "arm_1_robot_ip",
            default_value='172.17.0.3',
            description="IP address by which the robot can be reached.",
    )
    arm_1_controller_file_arg = DeclareLaunchArgument(
            "arm_1_controller_file",
            default_value="arm_1_ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
    )
    arm_1_tf_prefix_arg = DeclareLaunchArgument(
            "arm_1_tf_prefix",
            default_value="arm_1_",
            description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
    )
    arm_1_script_command_port_arg =  DeclareLaunchArgument(
            "arm_1_script_command_port",
            default_value="50010",
            description="Port that will be opened to forward script commands from the driver to the robot",
    )

    arm_1_trajectory_port_arg = DeclareLaunchArgument(
            "arm_1_trajectory_port",
            default_value="50009",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    arm_1_reverse_port_arg = DeclareLaunchArgument(
            "arm_1_reverse_port",
            default_value="50006",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    arm_1_script_sender_port_arg = DeclareLaunchArgument(
            "arm_1_script_sender_port",
            default_value="50007",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )


    

    ur_launch_dir = get_package_share_directory('ur_robot_driver')

    arm_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': ur_type,
                          'robot_ip': arm_0_robot_ip,
                          'controllers_file': arm_0_controller_file,
                          'tf_prefix': arm_0_tf_prefix,
                          'script_command_port': arm_0_script_command_port,
                          'trajectory_port': arm_0_trajectory_port,
                          'reverse_port': arm_0_reverse_port,
                          'script_sender_port': arm_0_script_sender_port,
                          }.items())
    
    arm_0_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_0'),
         arm_0
      ]
    )

    arm_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_launch_dir, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': ur_type,
                          'robot_ip': arm_1_robot_ip,
                          'controllers_file': arm_1_controller_file,
                          'tf_prefix': arm_1_tf_prefix,
                          'script_command_port': arm_1_script_command_port,
                          'trajectory_port': arm_1_trajectory_port,
                          'reverse_port': arm_1_reverse_port,
                          'script_sender_port': arm_1_script_sender_port,
                          }.items())
    
    arm_1_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_1'),
         arm_1
      ]
    )

    
    return LaunchDescription([
        ur_type_arg,
        arm_0_robot_ip_arg,
        arm_0_controller_file_arg,
        arm_0_tf_prefix_arg,
        arm_0_script_command_port_arg,
        arm_0_trajectory_port_arg,
        arm_0_reverse_port_arg,
        arm_0_script_sender_port_arg,
        arm_1_robot_ip_arg,
        arm_1_controller_file_arg,
        arm_1_tf_prefix_arg,
        arm_1_script_command_port_arg,
        arm_1_trajectory_port_arg,
        arm_1_reverse_port_arg,
        arm_1_script_sender_port_arg,

        # arm_0_with_namespace,
        arm_1_with_namespace
    ])
