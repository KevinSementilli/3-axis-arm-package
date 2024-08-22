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
    urdf_file = os.path.join(arm_project_pkg, 'robot_description', 'arm_project.urdf')
    controller_config_file = os.path.join(arm_project_pkg, 'config', 'joint_trajectory_controller.yaml')
    gazebo_launch_file = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'gui': 'true', 'server': 'true'}.items()
        )
    
    # Spawn the robot model in Gazebo
    spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            output='screen',
            arguments=[
                '-entity', 'three_axis_bot',
                '-x', LaunchConfiguration('arg_x'),
                '-y', LaunchConfiguration('arg_y'),
                '-z', LaunchConfiguration('arg_z'),
                '-Y', LaunchConfiguration('arg_Y'),
                '-param', 'robot_description',
                '-urdf', '-model', 'three_axis_bot',
                '-J', 'rotating_base_joint', '0.0',
                '-J', 'actuator_1_joint', '0.0',
                '-J', 'actuator_2_joint', '0.0'
            ]
        )
    
    # Launch the controller_manager node
    controller_launcher = Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[controller_config_file],
            output='screen',
        )
    
    # Spawn the controllers
    controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=['joint_state_controller', 'robot_arm_controller']
        )
    
    # Robot state publisher node
    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file, 'use_sim_time': True}]
        )

    # Launch description
    return LaunchDescription([
        declare_arg_x,
        declare_arg_y,
        declare_arg_z,
        declare_arg_R,
        declare_arg_P,
        declare_arg_Y,

        gazebo,
        spawn_entity,
        controller_launcher,
        controller_spawner,
        node_robot_state_publisher
    ])
