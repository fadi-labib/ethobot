import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_ethobot_simulation = get_package_share_directory('ethobot_simulation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file
    world_file = os.path.join(pkg_ethobot_simulation, 'worlds', 'ethobot_world.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    goal_x = LaunchConfiguration('goal_x', default='10.0')
    goal_y = LaunchConfiguration('goal_y', default='10.0')
    robot_x = LaunchConfiguration('robot_x', default='0.0')
    robot_y = LaunchConfiguration('robot_y', default='0.0')

    # Set TurtleBot3 model
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'goal_x',
            default_value='10.0',
            description='Goal X position'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='10.0',
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

        # Spawn TurtleBot3 in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': robot_x,
                'y_pose': robot_y,
            }.items()
        ),

        # ROS-Gazebo bridge for cmd_vel and odometry
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
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
                'max_iterations': 100,
                'publish_rate_hz': 2.0,
            }]
        ),

        # Swarm Visualizer Node
        Node(
            package='ethobot_algorithms',
            executable='swarm_visualizer_node',
            name='swarm_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_x': goal_x,
                'goal_y': goal_y,
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
                'goal_tolerance': 0.3,
                'max_linear_velocity': 0.15,
                'max_angular_velocity': 1.5,
            }]
        ),

        # Static transform for map frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),
    ])
