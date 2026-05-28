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


#include "ethobot_core/algorithm_base.hpp"

#include <chrono>
#include <thread>

namespace ethobot_core
{

AlgorithmBase::AlgorithmBase(const std::string & name)
: name_(name)
{
}

OptimizationResult AlgorithmBase::solve(
  std::size_t max_iterations,
  double target_fitness)
{
  running_ = true;
  paused_ = false;
  start_time_ = std::chrono::steady_clock::now();

  while (running_ && iteration_ < max_iterations) {
    if (paused_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    step();

    // Check convergence (for minimization)
    if (problem_.minimize && best_fitness_ <= target_fitness) {
      break;
    }
    // Check convergence (for maximization)
    if (!problem_.minimize && best_fitness_ >= target_fitness) {
      break;
    }
  }

  running_ = false;

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time_);

  OptimizationResult result;
  result.best_solution = best_solution_;
  result.best_fitness = best_fitness_;
  result.iterations = iteration_;
  result.elapsed_time_sec = duration.count() / 1000.0;
  result.converged = (problem_.minimize && best_fitness_ <= target_fitness) ||
    (!problem_.minimize && best_fitness_ >= target_fitness);

  return result;
}

ethobot_interfaces::msg::AlgorithmStatus AlgorithmBase::get_status() const
{
  ethobot_interfaces::msg::AlgorithmStatus status;
  status.algorithm_name = name_;

  if (running_) {
    status.status = paused_ ? "paused" : "running";
  } else if (iteration_ > 0) {
    status.status = "completed";
  } else {
    status.status = "idle";
  }

  status.current_iteration = static_cast<uint32_t>(iteration_);
  status.best_fitness = best_fitness_;

  if (running_) {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - start_time_);
    status.elapsed_time_sec = duration.count() / 1000.0;
  }

  return status;
}

}  // namespace ethobot_core
