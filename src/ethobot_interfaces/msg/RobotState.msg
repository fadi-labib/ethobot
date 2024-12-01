# Unified robot state across different robot types
std_msgs/Header header
string robot_id
string robot_type                  # "ground" or "aerial"
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
float64 battery_level              # 0.0 to 1.0
bool is_active
string current_task                # Description of current task
