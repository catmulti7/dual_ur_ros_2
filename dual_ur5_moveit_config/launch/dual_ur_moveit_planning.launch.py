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

    arm_0_prefix = LaunchConfiguration('arm_0_prefix') 
    arm_1_prefix = LaunchConfiguration('arm_0_prefix') 
    arm_0_controllers_file = LaunchConfiguration('arm_0_controllers_file') 
    arm_1_controllers_file = LaunchConfiguration('arm_1_controllers_file') 
    arm_0_planning_group = LaunchConfiguration('arm_0_planning_group') 
    arm_1_planning_group = LaunchConfiguration('arm_1_planning_group')
    if_use_rtde = LaunchConfiguration('if_use_rtde') 


    arm_0_prefix_arg = DeclareLaunchArgument(
            "arm_0_prefix",
            default_value='"arm_0"',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    arm_1_prefix_arg = DeclareLaunchArgument(
            "arm_1_prefix",
            default_value='"arm_1"',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    arm_0_controllers_file_arg = DeclareLaunchArgument(
            "arm_0_controllers_file",
            default_value="arm_0_controllers.yaml",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    arm_1_controllers_file_arg = DeclareLaunchArgument(
            "arm_1_controllers_file",
            default_value="arm_1_controllers.yaml",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
    )
    arm_0_planning_group_arg = DeclareLaunchArgument(
            "arm_0_planning_group",
            default_value="arm_0",
            description="planning group",
    )
    arm_1_planning_group_arg = DeclareLaunchArgument(
            "arm_1_planning_group",
            default_value="arm_1",
            description="planning group",
    )
    if_use_rtde_arg = DeclareLaunchArgument(
            "if_use_rtde",
            default_value="true",
            description="",
    )


    ur_moveit_launch_dir = get_package_share_directory('dual_ur5_moveit_config')

    arm_0_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', 'ur_moveit.launch.py')),
        launch_arguments={'prefix': arm_0_prefix,
                          'controllers_file': arm_0_controllers_file,
                          'planning_group' : arm_0_planning_group,
                          'if_use_rtde' : if_use_rtde
                          }.items())
    
    arm_0_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_0'),
         arm_0_moveit
      ]
    )

    arm_1_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', 'ur_moveit.launch.py')),
        launch_arguments={'prefix': arm_1_prefix,
                          'controllers_file': arm_1_controllers_file,
                          'planning_group' : arm_1_planning_group,
                          'if_use_rtde' : if_use_rtde
                          }.items())
    
    arm_1_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_1'),
         arm_1_moveit
      ]
    )

    
    return LaunchDescription([
        arm_0_prefix_arg,
        arm_0_controllers_file_arg,
        arm_0_planning_group_arg,
        arm_1_prefix_arg,
        arm_1_controllers_file_arg,
        arm_1_planning_group_arg,
        if_use_rtde_arg,
        #arm_0_moveit_with_namespace,
        arm_1_moveit_with_namespace
    ])
