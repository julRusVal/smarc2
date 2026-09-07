import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')
    num_robots = LaunchConfiguration('num_robots')
    
    config = os.path.join(
        get_package_share_directory('floatsam_controllers'),
        'config',
        'rvo.yaml'
    )

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='floatsam_usv'
    )

    use_sim_launch_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='False'
    )

    num_robots_launch_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='2'
    )

    node = Node(
        package='floatsam_controllers',
        executable='rvo_service_node',
        name='rvo_service_node',
        namespace=robot_ns,
        parameters=[
            config,
            {
                'robot_name': robot_ns,
                'use_sim': use_sim,
                'num_robots': num_robots,
            }
        ]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        use_sim_launch_arg,
        num_robots_launch_arg,
        node
    ])
