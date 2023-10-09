from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    keyboard = Node(
            package='receive_command',
            executable='receive_command',
            name='receive_command'
        )
    arm_0 = Node(
            package='robot_control',
            # namespace='arm_0',
            executable='robot_control',
            parameters=[
            {
                "arm_0/planning_group": 'arm_0',
                
            }]
        )
    arm_1 = Node(
            package='robot_control',
            namespace='arm_1',
            executable='robot_control',
            parameters=[
            {
                "arm_1/planning_group": 'arm_1',
            }]
        )
    
    arm_0_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('arm_0'),
         arm_0
      ]
    )
    
    
    return LaunchDescription([
        # keyboard,
        arm_0_with_namespace
        # arm_1
    ])