# Copyright 2026 Fadi Labib
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Simple launch file using TurtleBot3's built-in Gazebo world.

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

        # Static transform: map -> odom (robot spawns at origin in empty_world)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])
