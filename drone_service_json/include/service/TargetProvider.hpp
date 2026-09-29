#pragma once

#include "ScheduledWorker.hpp"
#include "service/interfaces/IConfigLoader.hpp"
#include "service/interfaces/ITargetProvider.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

class TargetProvider : public ITargetProvider, public ScheduledWorker {
  const std::shared_ptr<IConfigLoader> config_;

  // real-time spacing between the precomputed target positions (indexes the array + derives velocity)
  const float array_timestep_{};
  // wall-clock speed-up factor (simulated_time / real_time)
  const float timescale_{};

  std::vector<Target> targets_{};

  const int number_of_targets_{};
  const int number_of_positions_per_target_{};

  std::chrono::steady_clock::time_point start_{};
  bool started_{false};

  mutable std::mutex mtx_;

  void update_targets(const std::chrono::time_point<std::chrono::steady_clock>& start_,
                      const std::chrono::time_point<std::chrono::steady_clock>& now_);
  void tick() override;

public:
  TargetProvider(const std::shared_ptr<IConfigLoader> config)
    : ScheduledWorker(config->get_config().target_timestep / config->get_config().timescale)
    , config_(std::move(config))
    , array_timestep_(config_->get_config().target_array_timestep_)
    , timescale_(config_->get_config().timescale)
    , number_of_targets_(config_->get_targets().target_count_)
    , number_of_positions_per_target_(config_->get_targets().time_steps_)
  {
  }

  const Target get_target(int i) const override;
  std::vector<Target> get_targets() const override;
  int get_size() const override;
};
