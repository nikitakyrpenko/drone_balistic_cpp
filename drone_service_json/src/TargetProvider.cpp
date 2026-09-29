
#include "service/TargetProvider.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
#include <mutex>
#include <vector>
#include "models/Coord.hpp"
#include "models/Target.hpp"

const Target TargetProvider::get_target(int i) const
{
  std::lock_guard<std::mutex> l(mtx_);
  return targets_.at(i);
}

std::vector<Target> TargetProvider::get_targets() const
{
  std::lock_guard<std::mutex> l(mtx_);
  return targets_;
}

int TargetProvider::get_size() const
{
  return number_of_targets_;
}

void TargetProvider::update_targets(const std::chrono::time_point<std::chrono::steady_clock>& start,
                                    const std::chrono::time_point<std::chrono::steady_clock>& now)
{
  const float elapsed = std::chrono::duration<float>(now - start).count() * timescale_;
  const int curr_idx = static_cast<int>(std::floor(elapsed / array_timestep_)) % number_of_positions_per_target_;
  const int next_idx = (curr_idx + 1) % number_of_positions_per_target_;

  std::lock_guard<std::mutex> l(mtx_);

  targets_.clear();
  targets_.reserve(number_of_targets_);

  const auto& positions = config_->get_targets().positions_;
  for (int id = 0; id < static_cast<int>(positions.size()); ++id) {
    const std::vector<Coord>& coords = positions[id];
    const Coord& curr = coords.at(curr_idx);
    const Coord& next = coords.at(next_idx);
    targets_.push_back({id, curr, (next - curr) / array_timestep_});
  }
}

void TargetProvider::tick()
{
  if (!started_) {
    start_ = std::chrono::steady_clock::now();
    started_ = true;
  }
  update_targets(start_, std::chrono::steady_clock::now());
}
