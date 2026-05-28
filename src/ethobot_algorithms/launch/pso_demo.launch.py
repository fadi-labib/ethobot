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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ethobot_algorithms')
    rviz_config = os.path.join(pkg_share, 'rviz', 'pso_demo.rviz')
    obstacles_yaml = os.path.join(pkg_share, 'config', 'obstacles.yaml')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'goal_x', default_value='10.0',
            description='Goal X position'
        ),
        DeclareLaunchArgument(
            'goal_y', default_value='10.0',
            description='Goal Y position'
        ),
        DeclareLaunchArgument(
            'population_size', default_value='30',
            description='Number of PSO particles'
        ),
        DeclareLaunchArgument(
            'max_iterations', default_value='100',
            description='Maximum iterations'
        ),

        # PSO Path Planning Node
        Node(
            package='ethobot_algorithms',
            executable='pso_path_planning_node',
            name='pso_path_planning',
            output='screen',
            parameters=[obstacles_yaml, {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'population_size': LaunchConfiguration('population_size'),
                'max_iterations': LaunchConfiguration('max_iterations'),
                'publish_rate_hz': 2.0,  # Slower for better visualization
            }]
        ),

        # Swarm Visualizer Node
        Node(
            package='ethobot_algorithms',
            executable='swarm_visualizer_node',
            name='swarm_visualizer',
            output='screen',
            parameters=[obstacles_yaml, {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'particle_size': 0.3,
                'goal_size': 0.5,
            }]
        ),

        # Static transform for map frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
