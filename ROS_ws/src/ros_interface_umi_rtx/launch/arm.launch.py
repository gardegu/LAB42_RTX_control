import os
from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, ExecuteProcess

def generate_launch_description():
    
    cmd = "sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH PATH=$PATH USER=$USER ROS_DOMAIN_ID=$ROS_DOMAIN_ID $PWD/ROS_ws/install/ros_interface_umi_rtx/lib/ros_interface_umi_rtx/nodeArm"

    nodeArm = Node(
        package='ros_interface_umi_rtx',
        namespace='',
        executable='nodeArm',
        name='arm_node',
        output='screen'
    )
    
    nodeCamera = Node(
        package = 'ros_interface_umi_rtx',
        namespace='',
        executable='nodeCamera',
        name='control',
        output='screen'
    )
    
    nodeInvKin = Node(
        package = 'ros_interface_umi_rtx',
        namespace='',
        executable='nodeInverseKinematics',
        name='inverse_kinematics',
        output='screen'
    )
    
    return LaunchDescription([
                              nodeArm,  nodeInvKin, nodeCamera
                              ])


# executable='nodeArm',
# actions.ExecuteProcess(cmd=['ros2','bag','record','-a'],output='screen')

# nodeArm = ExecuteProcess(
#         cmd=['/usr/bin/bash', '-c', cmd],
#         output='screen'
#     )

                            #   set_sudo_env_var,

# nodeArm = Node(
#         package='ros_interface_umi_rtx',
#         namespace='',
#         executable='/usr/bin/bash',
#         name='arm_node',
#         shell=True,
#         arguments=['-c', cmd],
#         output='screen'
#     )
