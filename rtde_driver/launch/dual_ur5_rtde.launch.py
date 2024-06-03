from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    arm_0 = Node(
            package='rtde_driver',
            namespace='arm_0',
            executable='rtde_driver',
            output='screen',
            parameters=[
            {
                "ip_address": '172.17.0.2',
                "robot_name": 'arm_0'
                
            }]
        )
    arm_1 = Node(
            package='rtde_driver',
            namespace='arm_1',
            executable='rtde_driver',
            output='screen',
            parameters=[
            {
                "ip_address": '172.17.0.3',
                "robot_name": 'arm_1'
            }]
        )
    
    
    
    return LaunchDescription([
        arm_0,
        arm_1
    ])