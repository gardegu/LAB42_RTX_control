from launch import LaunchDescription, actions
from launch_ros.actions import Node
import os


def generate_launch_description():
    cwd = os.getcwd()
    os.chdir(cwd+"/logs/")
    
    nodeArm = Node(
        package='ros_interface_umi_rtx',
        namespace='',
        executable='nodeArm',
        name='arm_node',
        output='screen',
    )
    
    nodeCamera = Node(
        package = 'ros_interface_umi_rtx',
        namespace='',
        executable='nodeCamera',
        name='control',
    )
    
    nodeInvKin = Node(
        package = 'ros_interface_umi_rtx',
        namespace='',
        executable='nodeInverseKinematics',
        name='inverse_kinematics',
        output='screen'
    )
    
    return LaunchDescription([
                              nodeArm,  nodeInvKin, nodeCamera,
                              actions.ExecuteProcess(cmd=['ros2','bag','record','-a'],output='screen')
                              ])

## TODO : add this line in LaunchDescription to collect logs thanks to rosbag
# actions.ExecuteProcess(cmd=['ros2','bag','record','-a'],output='screen')