import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    declare_model_path_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory('arm_project'), 'urdf', 'arm_project.urdf'),
        description='Absolute path to robot urdf file'
    )

    # Define paths
    arm_project_pkg = get_package_share_directory('arm_project')
    urdf_file = LaunchConfiguration('model')
    rviz_config_file = os.path.join(arm_project_pkg, 'urdf.rviz')

    return LaunchDescription([
        declare_model_path_arg,

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
