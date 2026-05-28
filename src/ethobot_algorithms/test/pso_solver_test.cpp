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


#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ethobot_algorithms/pso_solver.hpp"
#include "ethobot_core/algorithm_base.hpp"

namespace ethobot_algorithms
{

// =============================================================================
// Test Fixtures
// =============================================================================

class PsoSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a simple 2D optimization problem: minimize sphere function
    // f(x, y) = x^2 + y^2
    // Global minimum at (0, 0) with fitness = 0
    problem_.dimensions = 2;
    problem_.lower_bounds = {-10.0, -10.0};
    problem_.upper_bounds = {10.0, 10.0};
    problem_.minimize = true;
    problem_.fitness_function = [](const std::vector<double> & x) {
        return x[0] * x[0] + x[1] * x[1];
      };
  }

  ethobot_core::Problem problem_;
};

// =============================================================================
// PsoParams Tests
// =============================================================================

TEST(PsoParamsTest, DefaultValues)
{
  PsoParams params;

  EXPECT_EQ(params.population_size, 30u);
  EXPECT_DOUBLE_EQ(params.inertia_weight, 0.7);
  EXPECT_DOUBLE_EQ(params.cognitive_coeff, 1.5);
  EXPECT_DOUBLE_EQ(params.social_coeff, 1.5);
  EXPECT_DOUBLE_EQ(params.max_velocity, 1.0);
}

TEST(PsoParamsTest, CustomValues)
{
  PsoParams params;
  params.population_size = 50;
  params.inertia_weight = 0.9;
  params.cognitive_coeff = 2.0;
  params.social_coeff = 2.0;
  params.max_velocity = 2.5;

  EXPECT_EQ(params.population_size, 50u);
  EXPECT_DOUBLE_EQ(params.inertia_weight, 0.9);
  EXPECT_DOUBLE_EQ(params.cognitive_coeff, 2.0);
  EXPECT_DOUBLE_EQ(params.social_coeff, 2.0);
  EXPECT_DOUBLE_EQ(params.max_velocity, 2.5);
}

// =============================================================================
// Particle Tests
// =============================================================================

TEST(ParticleTest, DefaultConstruction)
{
  Particle p;
  EXPECT_TRUE(p.position.empty());
  EXPECT_TRUE(p.velocity.empty());
  EXPECT_TRUE(p.personal_best_position.empty());
}

// =============================================================================
// PsoSolver Construction Tests
// =============================================================================

TEST_F(PsoSolverTest, DefaultConstruction)
{
  PsoSolver solver;

  EXPECT_EQ(solver.get_name(), "PSO");
  EXPECT_EQ(solver.get_iteration(), 0u);
  EXPECT_TRUE(solver.get_particles().empty());
}

TEST_F(PsoSolverTest, CustomParamsConstruction)
{
  PsoParams params;
  params.population_size = 50;
  params.inertia_weight = 0.8;

  PsoSolver solver(params);

  EXPECT_EQ(solver.get_params().population_size, 50u);
  EXPECT_DOUBLE_EQ(solver.get_params().inertia_weight, 0.8);
}

// =============================================================================
// PsoSolver Initialization Tests
// =============================================================================

TEST_F(PsoSolverTest, Initialize)
{
  PsoParams params;
  params.population_size = 20;
  PsoSolver solver(params);

  solver.initialize(problem_);

  // Verify particles created
  EXPECT_EQ(solver.get_particles().size(), 20u);

  // Verify all particles have correct dimensions
  for (const auto & particle : solver.get_particles()) {
    EXPECT_EQ(particle.position.size(), 2u);
    EXPECT_EQ(particle.velocity.size(), 2u);
    EXPECT_EQ(particle.personal_best_position.size(), 2u);
  }
}

TEST_F(PsoSolverTest, InitializeParticlesWithinBounds)
{
  PsoSolver solver;
  solver.initialize(problem_);

  for (const auto & particle : solver.get_particles()) {
    for (std::size_t d = 0; d < problem_.dimensions; ++d) {
      EXPECT_GE(particle.position[d], problem_.lower_bounds[d]);
      EXPECT_LE(particle.position[d], problem_.upper_bounds[d]);
    }
  }
}

TEST_F(PsoSolverTest, InitializeFitnessCalculated)
{
  PsoSolver solver;
  solver.initialize(problem_);

  for (const auto & particle : solver.get_particles()) {
    // Fitness should be calculated (not infinity for valid positions)
    EXPECT_LT(particle.fitness, std::numeric_limits<double>::infinity());
    EXPECT_GE(particle.fitness, 0.0);  // Sphere function is non-negative
  }
}

// =============================================================================
// PsoSolver Step Tests
// =============================================================================

TEST_F(PsoSolverTest, StepIncrementsIteration)
{
  PsoSolver solver;
  solver.initialize(problem_);

  EXPECT_EQ(solver.get_iteration(), 0u);

  solver.step();
  EXPECT_EQ(solver.get_iteration(), 1u);

  solver.step();
  EXPECT_EQ(solver.get_iteration(), 2u);
}

TEST_F(PsoSolverTest, StepUpdatesFitness)
{
  PsoSolver solver;
  solver.initialize(problem_);

  double initial_best = solver.get_best_fitness();

  // Run several iterations
  for (int i = 0; i < 10; ++i) {
    solver.step();
  }

  // Best fitness should not get worse (for minimization)
  EXPECT_LE(solver.get_best_fitness(), initial_best);
}

TEST_F(PsoSolverTest, ConvergenceTowardsOptimum)
{
  PsoParams params;
  params.population_size = 50;
  PsoSolver solver(params);
  solver.initialize(problem_);

  // Run many iterations
  for (int i = 0; i < 100; ++i) {
    solver.step();
  }

  // For sphere function, should converge close to (0, 0) with fitness near 0
  // Using a loose tolerance since PSO is stochastic
  EXPECT_LT(solver.get_best_fitness(), 1.0);
}

// =============================================================================
// PsoSolver Reset Tests
// =============================================================================

TEST_F(PsoSolverTest, Reset)
{
  PsoSolver solver;
  solver.initialize(problem_);

  // Run some iterations
  for (int i = 0; i < 10; ++i) {
    solver.step();
  }
  EXPECT_GT(solver.get_iteration(), 0u);

  solver.reset();

  EXPECT_EQ(solver.get_iteration(), 0u);
  EXPECT_TRUE(solver.get_particles().empty());
}

// =============================================================================
// PsoSolver SwarmState Tests
// =============================================================================

TEST_F(PsoSolverTest, GetSwarmState)
{
  PsoParams params;
  params.population_size = 10;
  PsoSolver solver(params);
  solver.initialize(problem_);

  auto state = solver.get_swarm_state();

  EXPECT_EQ(state.particles.size(), 10u);
  EXPECT_EQ(state.iteration, 0u);
}

TEST_F(PsoSolverTest, SwarmStateUpdatesAfterStep)
{
  PsoSolver solver;
  solver.initialize(problem_);
  solver.step();

  auto state = solver.get_swarm_state();

  EXPECT_EQ(state.iteration, 1u);
}

// =============================================================================
// PsoSolver Parameter Tuning Tests
// =============================================================================

TEST_F(PsoSolverTest, RuntimeParameterTuning)
{
  PsoSolver solver;

  solver.set_inertia_weight(0.9);
  EXPECT_DOUBLE_EQ(solver.get_params().inertia_weight, 0.9);

  solver.set_cognitive_coeff(2.0);
  EXPECT_DOUBLE_EQ(solver.get_params().cognitive_coeff, 2.0);

  solver.set_social_coeff(1.8);
  EXPECT_DOUBLE_EQ(solver.get_params().social_coeff, 1.8);
}

// =============================================================================
// TODO(fadi): Students should add more tests:
// - Test with Rastrigin function (multi-modal optimization)
// - Test with different dimensionality (1D, 3D, 10D)
// - Test velocity clamping behavior
// - Test boundary handling
// - Test with maximize = true
// - Test solve() with early termination
// - Compare convergence with different parameter settings
// - Test determinism with fixed random seed
// =============================================================================

}  // namespace ethobot_algorithms

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
