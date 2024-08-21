import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    declare_arg_x = DeclareLaunchArgument('arg_x', default_value='0.00')
    declare_arg_y = DeclareLaunchArgument('arg_y', default_value='0.00')
    declare_arg_z = DeclareLaunchArgument('arg_z', default_value='0.00')
    declare_arg_R = DeclareLaunchArgument('arg_R', default_value='0.00')
    declare_arg_P = DeclareLaunchArgument('arg_P', default_value='0.00')
    declare_arg_Y = DeclareLaunchArgument('arg_Y', default_value='0.00')

    # Define paths
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    arm_project_pkg = get_package_share_directory('arm_project')
    urdf_file = os.path.join(arm_project_pkg, 'urdf', 'arm_project.urdf')
    controller_config_file = os.path.join(arm_project_pkg, 'config', 'joint_trajectory_controller.yaml')

    # Launch description
    return LaunchDescription([
        declare_arg_x,
        declare_arg_y,
        declare_arg_z,
        declare_arg_R,
        declare_arg_P,
        declare_arg_Y,

        # Include the Gazebo empty world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'empty_world.launch.py'))
        ),

        # Static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40'],
        ),

        # Spawn the robot model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            output='screen',
            arguments=[
                '-x', LaunchConfiguration('arg_x'),
                '-y', LaunchConfiguration('arg_y'),
                '-z', LaunchConfiguration('arg_z'),
                '-Y', LaunchConfiguration('arg_Y'),
                '-param', 'robot_description',
                '-urdf', '-model', 'gr_007_v1',
                '-J', 'rotating_base_joint', '0.0',
                '-J', 'actuator_1_joint', '0.0',
                '-J', 'actuator_2_joint', '0.0'
            ]
        ),

        # Load the joint trajectory controller
        ExecuteProcess(
            cmd=['ros2', 'param', 'load', 'controller_manager', controller_config_file],
            output='screen'
        ),

        # Spawn the controllers
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=['joint_state_controller', 'robot_arm_controller']
        ),

        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        )
    ])
