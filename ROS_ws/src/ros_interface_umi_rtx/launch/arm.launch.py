import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch_ros.actions import Node

def generate_launch_description():
    nodeArm = Node(
        package='ros_interface_umi_rtx',
        namespace='',
        executable='nodeArm',
        name='arm_node'
    )
    
    return LaunchDescription([
                              nodeArm
                              ])

# actions.ExecuteProcess(cmd=['ros2','bag','record','-a'],output='screen'),
