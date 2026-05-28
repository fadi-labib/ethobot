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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_ethobot_simulation = get_package_share_directory('ethobot_simulation')
    pkg_ethobot_algorithms = get_package_share_directory('ethobot_algorithms')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file
    world_file = os.path.join(pkg_ethobot_simulation, 'worlds', 'ethobot_world.sdf')

    # Obstacle geometry shared by the PSO planner and the RViz visualizer.
    # Keep ethobot_world.sdf in sync with this file (see the note in the SDF).
    obstacles_yaml = os.path.join(pkg_ethobot_algorithms, 'config', 'obstacles.yaml')

    # Launch arguments - scaled for TurtleBot3 (4x4 meter space)
    goal_x = LaunchConfiguration('goal_x', default='3.0')
    goal_y = LaunchConfiguration('goal_y', default='3.0')
    robot_x = LaunchConfiguration('robot_x', default='0.0')
    robot_y = LaunchConfiguration('robot_y', default='0.0')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'goal_x',
            default_value='3.0',
            description='Goal X position'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='3.0',
            description='Goal Y position'
        ),
        DeclareLaunchArgument(
            'robot_x',
            default_value='0.0',
            description='Robot initial X position'
        ),
        DeclareLaunchArgument(
            'robot_y',
            default_value='0.0',
            description='Robot initial Y position'
        ),

        # Launch Gazebo with our world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}'
            }.items()
        ),

        # Spawn TurtleBot3 in Gazebo (includes its own ROS-Gazebo bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': robot_x,
                'y_pose': robot_y,
            }.items()
        ),

        # PSO Path Planning Node (scaled for 4x4 meter space)
        # PSO finds optimal WAYPOINT to avoid obstacles on path from start to goal
        Node(
            package='ethobot_algorithms',
            executable='pso_path_planning_node',
            name='pso_path_planning',
            output='screen',
            parameters=[obstacles_yaml, {
                'start_x': robot_x,
                'start_y': robot_y,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'population_size': 20,
                'max_iterations': 50,
                'publish_rate_hz': 5.0,
                'search_space_min': 0.0,
                'search_space_max': 4.0,
            }]
        ),

        # Swarm Visualizer Node
        Node(
            package='ethobot_algorithms',
            executable='swarm_visualizer_node',
            name='swarm_visualizer',
            output='screen',
            parameters=[obstacles_yaml, {
                'goal_x': goal_x,
                'goal_y': goal_y,
            }]
        ),

        # Waypoint Follower Node
        # Navigates: start → PSO waypoint → final goal.
        # spawn_x/spawn_y let the follower convert map-frame PSO waypoints
        # into the controller's odom frame (odom origin = robot spawn pose).
        Node(
            package='ethobot_robots',
            executable='waypoint_follower_node',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'goal_tolerance': 0.2,
                'max_linear_velocity': 0.15,
                'max_angular_velocity': 1.5,
                'final_goal_x': goal_x,
                'final_goal_y': goal_y,
                'spawn_x': robot_x,
                'spawn_y': robot_y,
            }]
        ),

        # Static transform: map -> odom.
        # TurtleBot3 initializes its odom frame at the spawn pose, so in the
        # map frame the odom origin sits at (robot_x, robot_y).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[robot_x, robot_y, '0', '0', '0', '0', 'map', 'odom']
        ),

        # RViz for particle visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_ethobot_simulation, 'config', 'gazebo_pso.rviz')],
            output='screen'
        ),
    ])
