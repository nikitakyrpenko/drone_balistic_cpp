#include "service/UartTargetProvider.hpp"
#include "ScheduledWorker.hpp"
#include "UartLink.hpp"

#include <iostream>
#include <utility>

UartTargetProvider::UartTargetProvider(std::shared_ptr<UartLink> link, std::shared_ptr<IConfigLoader> config_loader)
  : ScheduledWorker(config_loader->get_config().target_timestep / config_loader->get_config().timescale)
  , uart_link_(std::move(link))
  , config_loader_(std::move(config_loader))
  , target_channel_(uart_link_->target_channel())
{
}

void UartTargetProvider::update_target(const TimestampedTargetPos& tp)
{
  const int id = tp.pos.id;
  const Coord coord{tp.pos.x, tp.pos.y};

  std::lock_guard<std::mutex> l(mtx_);

  auto it = targets_.find(id);
  if (it == targets_.end()) {
    targets_.emplace(id, Target{id, coord, Coord{0.F, 0.F}});
    last_arrival_[id] = tp.arrival;
    return;
  }

  const float dt = std::chrono::duration<float>(tp.arrival - last_arrival_[id]).count();
  last_arrival_[id] = tp.arrival;
  if (dt <= 0.F) {
    return;
  }

  const Coord raw_velocity = (coord - it->second.pos_) / dt;
  const Coord velocity = (raw_velocity * VELOCITY_EMA_ALPHA) + (it->second.vel_ * (1.F - VELOCITY_EMA_ALPHA));

  it->second = Target{id, coord, velocity};
}

const Target UartTargetProvider::get_target(int id) const
{
  std::lock_guard<std::mutex> l(mtx_);
  return targets_.at(id);
}

std::vector<Target> UartTargetProvider::get_targets() const
{
  std::lock_guard<std::mutex> l(mtx_);

  std::vector<Target> out;
  out.reserve(targets_.size());
  for (const auto& [id, tracked] : targets_) {
    out.push_back(tracked);
  }

  std::cout << "[UARTLink] TARGETS:\n";
  for (const auto& t : out) {
    std::cout << " id=" << t.target_id_ << " (" << t.pos_.x_ << "," << t.pos_.y_ << ")\n";
  }
  std::cout << std::flush;

  return out;
}

int UartTargetProvider::get_size() const
{
  std::lock_guard<std::mutex> l(mtx_);
  return static_cast<int>(targets_.size());
}

void UartTargetProvider::tick()
{
  for (const auto& tp : target_channel_.drain_all()) {
    update_target(tp);
  }
}
