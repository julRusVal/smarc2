import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')
    
    config = os.path.join(
        get_package_share_directory('floatsam_controllers'),
        'config',
        'captain.yaml'
    )

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='floatsam_usv'
    )

    captain_node = Node(
        package='floatsam_controllers',
        namespace=robot_ns,
        executable='captain',
        name='captain',
        parameters=[config]  # Load parameters from the YAML file
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        captain_node
    ])