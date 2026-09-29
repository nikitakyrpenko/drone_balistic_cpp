#include "service/UartConfigLoader.hpp"

#include <optional>
#include <stdexcept>
#include <string>

UartConfigLoader::UartConfigLoader(std::shared_ptr<UartLink> link)
  : ScheduledWorker(0.01F)
  , link_(std::move(link))
{
}

void UartConfigLoader::tick()
{
  auto& ammo_q = link_->ammo_channel();
  auto& cfg_q = link_->config_channel();
  auto& tel_q = link_->telemetry_channel();

  if (!ammo_done_) {
    if (std::optional<dlink::AmmoCfg> ammo = ammo_q.drain_to_last()) {
      config_.ammo_ = std::string(ammo->name);
      config_.hit_radius_ = ammo->hitRadius;
      arsenal_.ammos_.push_back({std::string(ammo->name), ammo->mass, ammo->drag, ammo->lift});
      ammo_done_ = true;
      ready_.count_down();
    }
  }

  if (!cfg_done_) {
    if (std::optional<dlink::DroneCfg> cfg = cfg_q.drain_to_last()) {
      config_.attack_speed_ = cfg->attackSpeed;
      config_.acceleration_path_ = cfg->accelerationPath;
      config_.angular_speed_ = cfg->angularSpeed;
      config_.turn_threshold_ = cfg->turnThreshold;
      config_.time_step_ = cfg->timeStep;
      config_.timescale = cfg->timeScale;
      cfg_done_ = true;
      ready_.count_down();
    }
  }

  if (!telemetry_done_) {
    if (std::optional<dlink::Telemetry> t = tel_q.drain_to_last()) {
      config_.position_ = Coord{t->x, t->y};
      config_.initial_direction_ = t->dir;
      config_.altitude_ = t->z;
      telemetry_done_ = true;
      ready_.count_down();
    }
  }

  if (ammo_done_ && cfg_done_ && telemetry_done_) {
    interrupt();
  }
}

void UartConfigLoader::wait_ready() const
{
  ready_.wait();
}

const ConfigDTO& UartConfigLoader::get_config() const
{
  return config_;
}

const AmmoDTO& UartConfigLoader::get_arsenal() const
{
  return arsenal_;
}

const TargetDTO& UartConfigLoader::get_targets() const
{
  throw std::logic_error("not available in UART mode");
}
