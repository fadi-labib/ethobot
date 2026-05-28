// Copyright 2026 Fadi Labib
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#ifndef ETHOBOT_ALGORITHMS__PSO_SOLVER_HPP_
#define ETHOBOT_ALGORITHMS__PSO_SOLVER_HPP_

#include <random>
#include <vector>

#include "ethobot_core/algorithm_base.hpp"
#include "ethobot_interfaces/msg/particle_state.hpp"
#include "ethobot_interfaces/msg/swarm_state.hpp"

namespace ethobot_algorithms
{

/**
 * @brief PSO algorithm parameters
 */
struct PsoParams
{
  std::size_t population_size = 30;   // Number of particles
  double inertia_weight = 0.7;        // w: velocity inertia
  double cognitive_coeff = 1.5;       // c1: personal best attraction
  double social_coeff = 1.5;          // c2: global best attraction
  double max_velocity = 1.0;          // Maximum velocity per dimension
};

/**
 * @brief Particle in the swarm
 */
struct Particle
{
  std::vector<double> position;
  std::vector<double> velocity;
  double fitness;
  std::vector<double> personal_best_position;
  double personal_best_fitness;
};

/**
 * @brief Particle Swarm Optimization solver
 *
 * Classic PSO algorithm inspired by bird flocking behavior.
 * Each particle:
 *   1. Remembers its personal best position
 *   2. Knows the global best position (social learning)
 *   3. Updates velocity based on inertia + cognitive + social components
 *
 * Velocity update equation:
 *   v = w*v + c1*r1*(pbest - x) + c2*r2*(gbest - x)
 *
 * Position update:
 *   x = x + v
 */
class PsoSolver : public ethobot_core::AlgorithmBase
{
public:
  explicit PsoSolver(const PsoParams & params = PsoParams());
  ~PsoSolver() override = default;

  /**
   * @brief Initialize swarm with random positions
   * @param problem Problem definition
   */
  void initialize(const ethobot_core::Problem & problem) override;

  /**
   * @brief Execute one PSO iteration
   */
  void step() override;

  /**
   * @brief Reset swarm to initial state
   */
  void reset() override;

  /**
   * @brief Get swarm state for visualization
   * @return SwarmState message
   */
  ethobot_interfaces::msg::SwarmState get_swarm_state() const override;

  // Getters
  const std::vector<Particle> & get_particles() const {return particles_;}
  const PsoParams & get_params() const {return params_;}

  // Setters for runtime parameter tuning
  void set_inertia_weight(double w) {params_.inertia_weight = w;}
  void set_cognitive_coeff(double c1) {params_.cognitive_coeff = c1;}
  void set_social_coeff(double c2) {params_.social_coeff = c2;}

private:
  PsoParams params_;
  std::vector<Particle> particles_;
  std::vector<double> global_best_position_;
  double global_best_fitness_;

  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};

  /**
   * @brief Evaluate fitness for a particle
   */
  double evaluate(const std::vector<double> & position);

  /**
   * @brief Clamp position to bounds
   */
  void clamp_to_bounds(std::vector<double> & position);

  /**
   * @brief Clamp velocity to max
   */
  void clamp_velocity(std::vector<double> & velocity);
};

}  // namespace ethobot_algorithms

#endif  // ETHOBOT_ALGORITHMS__PSO_SOLVER_HPP_
