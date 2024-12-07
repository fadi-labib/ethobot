#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ethobot_interfaces/msg/swarm_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

/**
 * @brief Visualizes PSO swarm state in RViz
 *
 * Subscribes to swarm_state and publishes MarkerArray:
 * - Blue spheres: particles
 * - Green sphere: global best
 * - Red sphere: goal
 * - Gray cylinders: obstacles
 */
class SwarmVisualizerNode : public rclcpp::Node
{
public:
  SwarmVisualizerNode()
  : Node("swarm_visualizer")
  {
    // Parameters
    this->declare_parameter("goal_x", 10.0);
    this->declare_parameter("goal_y", 10.0);
    this->declare_parameter("particle_size", 0.3);
    this->declare_parameter("goal_size", 0.5);

    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    particle_size_ = this->get_parameter("particle_size").as_double();
    goal_size_ = this->get_parameter("goal_size").as_double();

    // Obstacles (same as in pso_path_planning_node)
    obstacles_ = {
      {3.0, 3.0, 1.5},
      {7.0, 4.0, 1.0},
      {5.0, 7.0, 1.2},
      {2.0, 8.0, 0.8}
    };

    // Publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "ethobot/visualization", 10);

    // Subscriber
    swarm_sub_ = this->create_subscription<ethobot_interfaces::msg::SwarmState>(
      "ethobot/swarm_state", 10,
      std::bind(&SwarmVisualizerNode::swarm_callback, this, _1));

    // Publish static markers periodically (obstacles, goal, ground)
    static_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SwarmVisualizerNode::publish_static_markers, this));

    RCLCPP_INFO(this->get_logger(), "Swarm Visualizer started");
    RCLCPP_INFO(this->get_logger(), "Goal: (%.1f, %.1f), Obstacles: %zu",
      goal_x_, goal_y_, obstacles_.size());
  }

private:
  void swarm_callback(const ethobot_interfaces::msg::SwarmState::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Add particle markers (reuse same IDs to avoid flickering)
    for (size_t i = 0; i < msg->particles.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "particles";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = msg->particles[i].position.x;
      marker.pose.position.y = msg->particles[i].position.y;
      marker.pose.position.z = 0.15;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = particle_size_;
      marker.scale.y = particle_size_;
      marker.scale.z = particle_size_;

      // Color based on fitness (blue = good, red = bad)
      double fitness_normalized = std::min(msg->particles[i].fitness / 5.0, 1.0);
      marker.color.r = static_cast<float>(fitness_normalized);
      marker.color.g = 0.3f;
      marker.color.b = static_cast<float>(1.0 - fitness_normalized);
      marker.color.a = 0.8f;

      markers.markers.push_back(marker);
    }

    // Global best marker (larger, green)
    visualization_msgs::msg::Marker best_marker;
    best_marker.header.frame_id = "map";
    best_marker.header.stamp = this->now();
    best_marker.ns = "global_best";
    best_marker.id = 0;
    best_marker.type = visualization_msgs::msg::Marker::SPHERE;
    best_marker.action = visualization_msgs::msg::Marker::ADD;

    best_marker.pose.position.x = msg->global_best.x;
    best_marker.pose.position.y = msg->global_best.y;
    best_marker.pose.position.z = 0.25;
    best_marker.pose.orientation.w = 1.0;

    best_marker.scale.x = particle_size_ * 1.5;
    best_marker.scale.y = particle_size_ * 1.5;
    best_marker.scale.z = particle_size_ * 1.5;

    best_marker.color.r = 0.0f;
    best_marker.color.g = 1.0f;
    best_marker.color.b = 0.0f;
    best_marker.color.a = 1.0f;

    markers.markers.push_back(best_marker);

    // Iteration text
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->now();
    text_marker.ns = "info";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = 6.0;
    text_marker.pose.position.y = -1.0;
    text_marker.pose.position.z = 1.0;
    text_marker.pose.orientation.w = 1.0;

    text_marker.scale.z = 0.5;

    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;

    char buf[128];
    snprintf(buf, sizeof(buf), "Iteration: %u/%u\nFitness: %.4f",
      msg->iteration, msg->max_iterations, msg->global_best_fitness);
    text_marker.text = buf;

    markers.markers.push_back(text_marker);

    marker_pub_->publish(markers);
  }

  void publish_static_markers()
  {
    visualization_msgs::msg::MarkerArray markers;

    // Goal marker (red sphere)
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = this->now();
    goal_marker.ns = "goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;

    goal_marker.pose.position.x = goal_x_;
    goal_marker.pose.position.y = goal_y_;
    goal_marker.pose.position.z = 0.25;
    goal_marker.pose.orientation.w = 1.0;

    goal_marker.scale.x = goal_size_;
    goal_marker.scale.y = goal_size_;
    goal_marker.scale.z = goal_size_;

    goal_marker.color.r = 1.0f;
    goal_marker.color.g = 0.2f;
    goal_marker.color.b = 0.2f;
    goal_marker.color.a = 1.0f;

    goal_marker.lifetime = rclcpp::Duration::from_seconds(0);  // Never expire

    markers.markers.push_back(goal_marker);

    // Obstacle markers (gray cylinders)
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      visualization_msgs::msg::Marker obs_marker;
      obs_marker.header.frame_id = "map";
      obs_marker.header.stamp = this->now();
      obs_marker.ns = "obstacles";
      obs_marker.id = static_cast<int>(i);
      obs_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      obs_marker.action = visualization_msgs::msg::Marker::ADD;

      obs_marker.pose.position.x = obstacles_[i][0];
      obs_marker.pose.position.y = obstacles_[i][1];
      obs_marker.pose.position.z = 0.5;
      obs_marker.pose.orientation.w = 1.0;

      double radius = obstacles_[i][2];
      obs_marker.scale.x = radius * 2.0;
      obs_marker.scale.y = radius * 2.0;
      obs_marker.scale.z = 1.0;

      obs_marker.color.r = 0.5f;
      obs_marker.color.g = 0.5f;
      obs_marker.color.b = 0.5f;
      obs_marker.color.a = 0.7f;

      obs_marker.lifetime = rclcpp::Duration::from_seconds(0);

      markers.markers.push_back(obs_marker);
    }

    // Ground plane
    visualization_msgs::msg::Marker ground_marker;
    ground_marker.header.frame_id = "map";
    ground_marker.header.stamp = this->now();
    ground_marker.ns = "ground";
    ground_marker.id = 0;
    ground_marker.type = visualization_msgs::msg::Marker::CUBE;
    ground_marker.action = visualization_msgs::msg::Marker::ADD;

    ground_marker.pose.position.x = 6.0;
    ground_marker.pose.position.y = 6.0;
    ground_marker.pose.position.z = -0.05;
    ground_marker.pose.orientation.w = 1.0;

    ground_marker.scale.x = 14.0;
    ground_marker.scale.y = 14.0;
    ground_marker.scale.z = 0.1;

    ground_marker.color.r = 0.2f;
    ground_marker.color.g = 0.2f;
    ground_marker.color.b = 0.3f;
    ground_marker.color.a = 1.0f;

    ground_marker.lifetime = rclcpp::Duration::from_seconds(0);

    markers.markers.push_back(ground_marker);

    marker_pub_->publish(markers);
  }

  // Publishers/Subscribers/Timers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<ethobot_interfaces::msg::SwarmState>::SharedPtr swarm_sub_;
  rclcpp::TimerBase::SharedPtr static_timer_;

  // Parameters
  double goal_x_;
  double goal_y_;
  double particle_size_;
  double goal_size_;
  std::vector<std::vector<double>> obstacles_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SwarmVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
