# Command to control swarm behavior
string command_type                # "start", "stop", "reset", "pause", "resume"
string algorithm                   # "pso", "aco", "ga", ""
string[] target_robots             # Empty means all robots
geometry_msgs/Point[] waypoints    # Optional waypoints for navigation
