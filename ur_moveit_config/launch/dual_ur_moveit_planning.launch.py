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


    ur_moveit_launch_dir = get_package_share_directory('ur_moveit_config')

    arm_0_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', 'ur_moveit.launch.py')))
    
    arm_0_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_0'),
         arm_0_moveit
      ]
    )

    arm_1_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', 'ur_moveit.launch.py')))
    
    arm_1_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_1'),
         arm_1_moveit
      ]
    )

    
    return LaunchDescription([
        # arm_0_moveit_with_namespace,
        arm_1_moveit_with_namespace
    ])
