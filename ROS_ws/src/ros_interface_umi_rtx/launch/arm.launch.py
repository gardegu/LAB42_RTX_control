from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
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
                              nodeArm,  nodeInvKin, nodeCamera
                              ])


# nodeArm = Node(
#         package='ros_interface_umi_rtx',
#         namespace='',
#         executable='/usr/bin/bash',
#         name='arm_node',
#         shell=True,
#         arguments=['-c', cmd],
#         output='screen'
#     )


# nodeArm = Node(
#         package='ros_interface_umi_rtx',
#         namespace='',
#         executable='nodeArm',
#         name='arm_node',
#         output='screen',
#         prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  /bin/bash -c "]
#     )
