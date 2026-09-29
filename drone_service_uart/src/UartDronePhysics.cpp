#include "service/UartDronePhysics.hpp"
#include "ScheduledWorker.hpp"
#include "UartLink.hpp"
#include "service/interfaces/IConfigLoader.hpp"
#include "service/state/StateStopped.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

UartDronePhysics::UartDronePhysics(std::shared_ptr<UartLink> uart_link, std::shared_ptr<IConfigLoader> config_loader)
  : ScheduledWorker(config_loader->get_config().physics_timestep / config_loader->get_config().timescale)
  , uart_link_(std::move(uart_link))
  , config_loader_(std::move(config_loader))
  , telemetry_channel_(uart_link_->telemetry_channel())
  , telemetry_(
      config_loader_->get_config().position_, config_loader_->get_config().initial_direction_, config_loader_->get_config().altitude_)
  , spec_(config_loader_->get_config().attack_speed_,
          config_loader_->get_config().acceleration_path_,
          config_loader_->get_config().angular_speed_,
          config_loader_->get_config().turn_threshold_)
{
}

const IState* UartDronePhysics::state_from_mode(DroneMode mode)
{
  switch (mode) {
    case DroneMode::ACCELERATING:
      return StateAccelerating::get_instance();
    case DroneMode::DECELERATING:
      return StateDecelerating::get_instance();
    case DroneMode::TURNING:
      return StateTurning::get_instance();
    case DroneMode::MOVING:
      return StateMoving::get_instance();
    default:
      return StateStopped::get_instance();
  }
}

void UartDronePhysics::apply(const dlink::Telemetry& t)
{
  std::lock_guard<std::mutex> l(tel_mtx_);
  telemetry_.set_position(Coord{t.x, t.y});
  telemetry_.set_altitude(t.z);
  telemetry_.set_current_speed(t.speed);
  telemetry_.set_current_direction(t.dir);
  telemetry_.set_elapsed(static_cast<float>(t.t_ms) / 1000.0F);
}

dlink::Control UartDronePhysics::to_control(const DroneCommand& cmd) const
{
  std::lock_guard<std::mutex> l(tel_mtx_);
  const float delta =
    std::atan2(std::sin(cmd.dir - telemetry_.get_current_direction()), std::cos(cmd.dir - telemetry_.get_current_direction()));

  const float accel = (cmd.state == DroneMode::ACCELERATING || cmd.state == DroneMode::MOVING) ? 1.0F : -1.0F;

  const float turn_magnitude = spec_.get_angular_speed() * ScheduledWorker::idle_;
  const float turn_rate = std::clamp(delta / turn_magnitude, -1.0F, 1.0F);

  return {accel, turn_rate};
}

void UartDronePhysics::tick()
{
  if (auto t = telemetry_channel_.drain_to_last()) {
    apply(*t);
  }
  if (auto cmd = command_channel_.drain_to_last()) {
    std::lock_guard<std::mutex> l(command_mtx_);
    command_ = *cmd;
  }

  uart_link_->send(to_control(get_active_command()));
}

void UartDronePhysics::submit_command(const DroneCommand& command) const
{
  command_channel_.emplace(command);
}

const DroneTelemetry UartDronePhysics::get_telemetry() const
{
  std::lock_guard<std::mutex> l(tel_mtx_);
  return telemetry_;
}

const DroneSpec UartDronePhysics::get_spec() const
{
  return spec_;
}

const DroneCommand UartDronePhysics::get_active_command() const
{
  std::lock_guard<std::mutex> l(command_mtx_);
  return command_;
}

const IState* UartDronePhysics::get_state() const
{
  return state_from_mode(get_active_command().state);
}
