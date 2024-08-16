import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    arm_project_pkg = get_package_share_directory('arm_project')

    # Define the paths to the launch and URDF files
    empty_world_launch = os.path.join(gazebo_ros_pkg, 'launch', 'empty_world.launch.py')
    urdf_file = os.path.join(arm_project_pkg, 'urdf', 'arm_project.urdf')

    return LaunchDescription([
        # Include the empty_world launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(empty_world_launch)
        ),
        # Static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
        ),
        # Spawn model node
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=['-file', urdf_file, '-urdf', '-model', 'arm_project'],
            output='screen'
        ),
        # Fake joint calibration node
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', 'data: true'],
            name='fake_joint_calibration',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()