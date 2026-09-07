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

    #Values from the bringup
    robot_name = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')
    
    #Picking the correct config file
    config_dir = os.path.join(get_package_share_directory('floatsam_loiter_heading'), 'config')

    config_file_name = PythonExpression([
        "'loiter_heading_params_sim.yaml' if '", use_sim, "'.lower() == 'true' else 'loiter_heading_params_real.yaml'"
    ])
    config_file = PathJoinSubstitution([config_dir, config_file_name])

    #Node configuration
    node = Node(
        package='floatsam_loiter_heading',
        executable='floatsam_loiter_heading_action_server',
        name='floatsam_loiter_heading_action_server',
        namespace=robot_name,
        output='screen',
        parameters=[
            config_file,
            {
                'robot_name': robot_name,
                'use_sim': use_sim,
            }
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        node
    ])