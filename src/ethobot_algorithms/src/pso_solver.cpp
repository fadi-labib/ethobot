#include "ethobot_algorithms/pso_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ethobot_algorithms
{

PsoSolver::PsoSolver(const PsoParams & params)
: AlgorithmBase("PSO"),
  params_(params),
  global_best_fitness_(std::numeric_limits<double>::infinity()),
  rng_(std::random_device{}())
{
}

void PsoSolver::initialize(const ethobot_core::Problem & problem)
{
  problem_ = problem;
  iteration_ = 0;
  best_fitness_ = std::numeric_limits<double>::infinity();
  global_best_fitness_ = std::numeric_limits<double>::infinity();

  const std::size_t dims = problem_.dimensions;

  // Initialize particles with random positions and velocities
  particles_.clear();
  particles_.resize(params_.population_size);

  for (auto & particle : particles_) {
    particle.position.resize(dims);
    particle.velocity.resize(dims);
    particle.personal_best_position.resize(dims);

    // Random position within bounds
    for (std::size_t d = 0; d < dims; ++d) {
      double range = problem_.upper_bounds[d] - problem_.lower_bounds[d];
      particle.position[d] = problem_.lower_bounds[d] + uniform_dist_(rng_) * range;

      // Random velocity (small fraction of range)
      particle.velocity[d] = (uniform_dist_(rng_) - 0.5) * range * 0.1;
    }

    // Evaluate initial fitness
    particle.fitness = evaluate(particle.position);
    particle.personal_best_position = particle.position;
    particle.personal_best_fitness = particle.fitness;

    // Update global best
    if (particle.fitness < global_best_fitness_) {
      global_best_fitness_ = particle.fitness;
      global_best_position_ = particle.position;
    }
  }

  best_fitness_ = global_best_fitness_;
  best_solution_ = global_best_position_;
}

void PsoSolver::step()
{
  const std::size_t dims = problem_.dimensions;

  for (auto & particle : particles_) {
    // Update velocity for each dimension
    for (std::size_t d = 0; d < dims; ++d) {
      double r1 = uniform_dist_(rng_);
      double r2 = uniform_dist_(rng_);

      // PSO velocity update equation:
      // v = w*v + c1*r1*(pbest - x) + c2*r2*(gbest - x)
      double cognitive = params_.cognitive_coeff * r1 *
        (particle.personal_best_position[d] - particle.position[d]);

      double social = params_.social_coeff * r2 *
        (global_best_position_[d] - particle.position[d]);

      particle.velocity[d] = params_.inertia_weight * particle.velocity[d] +
        cognitive + social;
    }

    // Clamp velocity
    clamp_velocity(particle.velocity);

    // Update position: x = x + v
    for (std::size_t d = 0; d < dims; ++d) {
      particle.position[d] += particle.velocity[d];
    }

    // Clamp position to bounds
    clamp_to_bounds(particle.position);

    // Evaluate new fitness
    particle.fitness = evaluate(particle.position);

    // Update personal best
    if (particle.fitness < particle.personal_best_fitness) {
      particle.personal_best_fitness = particle.fitness;
      particle.personal_best_position = particle.position;
    }

    // Update global best
    if (particle.fitness < global_best_fitness_) {
      global_best_fitness_ = particle.fitness;
      global_best_position_ = particle.position;
    }
  }

  // Update algorithm state
  best_fitness_ = global_best_fitness_;
  best_solution_ = global_best_position_;
}

void PsoSolver::reset()
{
  particles_.clear();
  global_best_position_.clear();
  global_best_fitness_ = std::numeric_limits<double>::infinity();
  iteration_ = 0;
  best_fitness_ = std::numeric_limits<double>::infinity();
  best_solution_.clear();
}

ethobot_interfaces::msg::SwarmState PsoSolver::get_swarm_state() const
{
  ethobot_interfaces::msg::SwarmState state;
  state.header.stamp = rclcpp::Clock().now();
  state.iteration = static_cast<uint32_t>(iteration_);
  state.global_best_fitness = global_best_fitness_;
  state.converged = false;  // Could add convergence detection

  // Set global best position (assuming 2D or 3D)
  if (!global_best_position_.empty()) {
    state.global_best.x = global_best_position_[0];
    state.global_best.y = (global_best_position_.size() > 1) ? global_best_position_[1] : 0.0;
    state.global_best.z = (global_best_position_.size() > 2) ? global_best_position_[2] : 0.0;
  }

  // Convert particles to messages
  state.particles.reserve(particles_.size());
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    ethobot_interfaces::msg::ParticleState ps;
    ps.id = static_cast<uint32_t>(i);

    // Position (assuming 2D or 3D)
    ps.position.x = particles_[i].position[0];
    ps.position.y = (particles_[i].position.size() > 1) ? particles_[i].position[1] : 0.0;
    ps.position.z = (particles_[i].position.size() > 2) ? particles_[i].position[2] : 0.0;

    // Velocity
    ps.velocity.x = particles_[i].velocity[0];
    ps.velocity.y = (particles_[i].velocity.size() > 1) ? particles_[i].velocity[1] : 0.0;
    ps.velocity.z = (particles_[i].velocity.size() > 2) ? particles_[i].velocity[2] : 0.0;

    ps.fitness = particles_[i].fitness;

    // Personal best
    ps.personal_best.x = particles_[i].personal_best_position[0];
    ps.personal_best.y = (particles_[i].personal_best_position.size() > 1) ?
      particles_[i].personal_best_position[1] : 0.0;
    ps.personal_best.z = (particles_[i].personal_best_position.size() > 2) ?
      particles_[i].personal_best_position[2] : 0.0;
    ps.personal_best_fitness = particles_[i].personal_best_fitness;

    state.particles.push_back(ps);
  }

  return state;
}

double PsoSolver::evaluate(const std::vector<double> & position)
{
  if (problem_.fitness_function) {
    return problem_.fitness_function(position);
  }
  return std::numeric_limits<double>::infinity();
}

void PsoSolver::clamp_to_bounds(std::vector<double> & position)
{
  for (std::size_t d = 0; d < position.size(); ++d) {
    position[d] = std::clamp(
      position[d],
      problem_.lower_bounds[d],
      problem_.upper_bounds[d]);
  }
}

void PsoSolver::clamp_velocity(std::vector<double> & velocity)
{
  for (auto & v : velocity) {
    v = std::clamp(v, -params_.max_velocity, params_.max_velocity);
  }
}

}  // namespace ethobot_algorithms
