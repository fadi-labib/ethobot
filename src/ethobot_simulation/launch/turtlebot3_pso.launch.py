"""
Simple launch file using TurtleBot3's built-in Gazebo world.
Use this if the custom world has issues.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    goal_x = LaunchConfiguration('goal_x', default='2.0')
    goal_y = LaunchConfiguration('goal_y', default='2.0')

    return LaunchDescription([
        # Set TurtleBot3 model environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='2.0',
            description='Goal X position'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='2.0',
            description='Goal Y position'
        ),

        # Launch TurtleBot3 in empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'empty_world.launch.py')
            ),
        ),

        # PSO Path Planning Node
        Node(
            package='ethobot_algorithms',
            executable='pso_path_planning_node',
            name='pso_path_planning',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'population_size': 30,
                'max_iterations': 50,
                'publish_rate_hz': 2.0,
            }]
        ),

        # Waypoint Follower Node
        Node(
            package='ethobot_robots',
            executable='waypoint_follower_node',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_tolerance': 0.2,
                'max_linear_velocity': 0.15,
                'max_angular_velocity': 1.5,
            }]
        ),

        # Static transform for map frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
        ),
    ])
