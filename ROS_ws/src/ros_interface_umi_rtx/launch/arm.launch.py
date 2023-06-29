from launch import LaunchDescription, actions
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


import os


def generate_launch_description():
    cwd = os.getcwd()
    os.chdir(cwd+"/logs/")
    
    pkg_share = FindPackageShare(package='ros_interface_umi_rtx').find('ros_interface_umi_rtx')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/umi_rtx.urdf')
    
    # Launch configuration variables specific to simulation
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf_model])}],
        arguments=[default_urdf_model_path])
    
    # Launch RViz
    nodeRviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file])
    
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
    
    nodeSimu = Node(
        package = 'ros_interface_umi_rtx',
        namespace='',
        executable='nodeSimu',
        name='simulation',
        output='screen'
    )
    
    GUI = Node(
        package = 'ros_interface_umi_rtx',
        namespace='',
        executable='GUI',
        name='GUI',
    )
    
    return LaunchDescription([declare_rviz_config_file_cmd, declare_use_robot_state_pub_cmd,
                              declare_use_sim_time_cmd, declare_urdf_model_path_cmd,
                              start_robot_state_publisher_cmd,
                              nodeArm, nodeInvKin,
                              nodeSimu,
                              nodeCamera,
                              GUI
                              ])

                            #   actions.ExecuteProcess(cmd=["rqt"])
    #nodeCamera, actions.ExecuteProcess(cmd=['ros2','bag','record','-a'],output='screen') #TODO
