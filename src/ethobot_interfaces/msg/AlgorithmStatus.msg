# Status of running algorithm
std_msgs/Header header
string algorithm_name              # "pso", "aco", "ga"
string status                      # "idle", "running", "paused", "completed", "failed"
uint32 current_iteration
uint32 max_iterations
float64 best_fitness
float64 convergence_rate           # Rate of improvement
float64 elapsed_time_sec
string[] active_robots
