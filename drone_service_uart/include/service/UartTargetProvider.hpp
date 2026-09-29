#pragma once

#include "ScheduledWorker.hpp"
#include "UartLink.hpp"
#include "service/interfaces/IConfigLoader.hpp"
#include "service/interfaces/ITargetProvider.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>

class UartTargetProvider : public ITargetProvider, public ScheduledWorker {
private:
  static constexpr float VELOCITY_EMA_ALPHA = 0.3F;

  std::shared_ptr<UartLink> uart_link_;
  std::shared_ptr<IConfigLoader> config_loader_;

  SynchronizedQueue<TimestampedTargetPos>& target_channel_;

  std::map<int, Target> targets_{};
  std::map<int, std::chrono::steady_clock::time_point> last_arrival_{};
  mutable std::mutex mtx_;

  void update_target(const TimestampedTargetPos& tp);
  void tick() override;

public:
  explicit UartTargetProvider(std::shared_ptr<UartLink> uart_link, std::shared_ptr<IConfigLoader> config_loader);

  const Target get_target(int id) const override;
  std::vector<Target> get_targets() const override;

  int get_size() const override;
};
