import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

WORLD_FILE_NAME = os.getenv("WORLD_FILE_NAME", "dance.world")
ROBOT_ID = os.getenv("ROBOT_ID", "jetbot")

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    pkg_gazebo_dance_world = get_package_share_directory('gazebo_dance_world')
    
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='jetbot'),
        DeclareLaunchArgument('robot_model', default_value='simple_diff_ros'),
        DeclareLaunchArgument('x', default_value='-0.3'),
        DeclareLaunchArgument('y', default_value='-2.65'),
        DeclareLaunchArgument('z', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_dance_world, 'launch', 'dance.launch.py')
            )
        ),
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),
        Node(package='jetbot_ros', executable='gazebo_spawn', 
                        parameters=[
                            {'name': LaunchConfiguration('robot_name')},
                            {'model': LaunchConfiguration('robot_model')},
                            {'x': LaunchConfiguration('x')},
                            {'y': LaunchConfiguration('y')},
                            {'z': LaunchConfiguration('z')},
                        ],
                        output='screen', emulate_tty=True),
        Node(package='jetbot_dance_simulation', executable='jetbot_move_direction', namespace=ROBOT_ID,
                        output='screen', emulate_tty=True)
    ])