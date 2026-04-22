import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')
    config = os.path.join(
        get_package_share_directory('floatsam_controllers'),
        'config',
        'rvo.yaml'
    )

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='floatsam_usv'
    )

    node = Node(
            package='floatsam_controllers',
            executable='rvo_service_node',
            name='rvo_service_node',
            namespace=robot_ns,
            parameters=[config]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        node
    ])