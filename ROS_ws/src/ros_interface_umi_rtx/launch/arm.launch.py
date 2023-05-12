import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node

def generate_launch_description():
    
    cmd = "sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH PATH=$PATH USER=$USER ROS_DOMAIN_ID=$ROS_DOMAIN_ID $PWD/ROS_ws/install/ros_interface_umi_rtx/lib/ros_interface_umi_rtx/nodeArm"
    
    nodeArm = Node(
        package='ros_interface_umi_rtx',
        namespace='',
        executable='/usr/bin/bash',
        name='arm_node',
        arguments=['-c', cmd],
        output='screen'
    )
    
    return LaunchDescription([
                              nodeArm
                              ])

# executable='nodeArm',
# actions.ExecuteProcess(cmd=['ros2','bag','record','-a'],output='screen'),
# prefix="sudo -E env PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH PATH=$PATH USER=$USER"
# cmd = "sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH PATH=$PATH USER=$USER "\
#           "$PWD/ROS_ws/install/ros_interface_umi_rtx/lib/ros_interface_umi_rtx/nodeArm"