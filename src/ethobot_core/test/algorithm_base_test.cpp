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
#include <vector>

#include "ethobot_core/algorithm_base.hpp"

namespace ethobot_core
{

// Concrete implementation for testing the abstract base class
class TestAlgorithm : public AlgorithmBase
{
public:
  TestAlgorithm()
  : AlgorithmBase("TestAlgorithm") {}

  void initialize(const Problem & problem) override
  {
    problem_ = problem;
    iteration_ = 0;
    best_fitness_ = std::numeric_limits<double>::infinity();
    best_solution_.resize(problem.dimensions, 0.0);
  }

  void step() override
  {
    ++iteration_;
    // Simple test: move toward origin
    for (auto & val : best_solution_) {
      val *= 0.9;
    }
    best_fitness_ = 0.0;
    for (const auto & val : best_solution_) {
      best_fitness_ += val * val;
    }
  }

  void reset() override
  {
    iteration_ = 0;
    best_fitness_ = std::numeric_limits<double>::infinity();
    best_solution_.clear();
  }

  ethobot_interfaces::msg::SwarmState get_swarm_state() const override
  {
    return ethobot_interfaces::msg::SwarmState();
  }
};

// =============================================================================
// Problem Struct Tests
// =============================================================================

TEST(ProblemTest, DefaultConstruction)
{
  Problem problem;
  problem.dimensions = 2;
  problem.lower_bounds = {-10.0, -10.0};
  problem.upper_bounds = {10.0, 10.0};
  problem.minimize = true;

  EXPECT_EQ(problem.dimensions, 2u);
  EXPECT_EQ(problem.lower_bounds.size(), 2u);
  EXPECT_EQ(problem.upper_bounds.size(), 2u);
  EXPECT_TRUE(problem.minimize);
}

TEST(ProblemTest, FitnessFunctionAssignment)
{
  Problem problem;
  problem.dimensions = 2;
  problem.fitness_function = [](const std::vector<double> & x) {
      return x[0] * x[0] + x[1] * x[1];  // Sphere function
    };

  std::vector<double> test_point = {3.0, 4.0};
  double fitness = problem.fitness_function(test_point);

  EXPECT_DOUBLE_EQ(fitness, 25.0);  // 3^2 + 4^2 = 25
}

// =============================================================================
// OptimizationResult Tests
// =============================================================================

TEST(OptimizationResultTest, DefaultValues)
{
  OptimizationResult result;
  result.best_solution = {1.0, 2.0};
  result.best_fitness = 5.0;
  result.iterations = 100;
  result.elapsed_time_sec = 1.5;
  result.converged = true;

  EXPECT_EQ(result.best_solution.size(), 2u);
  EXPECT_DOUBLE_EQ(result.best_fitness, 5.0);
  EXPECT_EQ(result.iterations, 100u);
  EXPECT_TRUE(result.converged);
}

// =============================================================================
// AlgorithmBase Tests
// =============================================================================

TEST(AlgorithmBaseTest, GetName)
{
  TestAlgorithm algo;
  EXPECT_EQ(algo.get_name(), "TestAlgorithm");
}

TEST(AlgorithmBaseTest, InitialState)
{
  TestAlgorithm algo;
  EXPECT_EQ(algo.get_iteration(), 0u);
  EXPECT_FALSE(algo.is_running());
}

TEST(AlgorithmBaseTest, Initialize)
{
  TestAlgorithm algo;

  Problem problem;
  problem.dimensions = 3;
  problem.lower_bounds = {-5.0, -5.0, -5.0};
  problem.upper_bounds = {5.0, 5.0, 5.0};

  algo.initialize(problem);

  EXPECT_EQ(algo.get_iteration(), 0u);
  EXPECT_EQ(algo.get_best_solution().size(), 3u);
}

TEST(AlgorithmBaseTest, Step)
{
  TestAlgorithm algo;

  Problem problem;
  problem.dimensions = 2;
  problem.lower_bounds = {-10.0, -10.0};
  problem.upper_bounds = {10.0, 10.0};

  algo.initialize(problem);
  EXPECT_EQ(algo.get_iteration(), 0u);

  algo.step();
  EXPECT_EQ(algo.get_iteration(), 1u);

  algo.step();
  EXPECT_EQ(algo.get_iteration(), 2u);
}

TEST(AlgorithmBaseTest, Reset)
{
  TestAlgorithm algo;

  Problem problem;
  problem.dimensions = 2;
  problem.lower_bounds = {-10.0, -10.0};
  problem.upper_bounds = {10.0, 10.0};

  algo.initialize(problem);
  algo.step();
  algo.step();
  EXPECT_EQ(algo.get_iteration(), 2u);

  algo.reset();
  EXPECT_EQ(algo.get_iteration(), 0u);
  EXPECT_TRUE(algo.get_best_solution().empty());
}

TEST(AlgorithmBaseTest, PauseResume)
{
  TestAlgorithm algo;

  algo.pause();
  // Pause/resume are simple state changes, verify no crash
  algo.resume();
  algo.stop();

  EXPECT_FALSE(algo.is_running());
}

// =============================================================================
// TODO(fadi): Students should add more tests:
// - Test solve() method with convergence criteria
// - Test get_status() returns valid AlgorithmStatus message
// - Test behavior with edge cases (0 dimensions, empty bounds)
// - Test thread safety of pause/resume/stop
// =============================================================================

}  // namespace ethobot_core

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
