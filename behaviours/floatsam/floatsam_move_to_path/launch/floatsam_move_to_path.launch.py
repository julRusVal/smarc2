import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression, PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #Fallback values
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', 
        default_value='floatsam_usv',
        description='Namespace for the robot'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', 
        default_value='true',  
        description='Flag to enable simulation-specific configurations'
    )

    robot_name = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')

    config_dir = os.path.join(get_package_share_directory('floatsam_move_to_path'), 'config')

    config_file_name = PythonExpression([
        "'move_to_path_params_sim.yaml' if '", use_sim, "'.lower() == 'true' else 'move_to_path_params_real.yaml'"
    ])
    config_file = PathJoinSubstitution([config_dir, config_file_name])


    node = Node(
        package='floatsam_move_to_path',
        executable='floatsam_move_to_path_action_server',
        name='floatsam_move_to_path_action_server',
        namespace=robot_name,
        parameters=[
            config_file,
            {
                'robot_name': robot_name,
                'use_sim': use_sim,
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        node
    ])