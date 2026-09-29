#include "service/DronePhysics.hpp"
#include "service/state/StateStopped.hpp"

#include <mutex>

static const IState* state_from_mode(DroneMode mode)
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

void DronePhysics::step(float dt)
{
  // calculate direction
  float delta =
    std::atan2(std::sin(command_.dir - telemetry_.get_current_direction()), std::cos(command_.dir - telemetry_.get_current_direction()));

  float rot_step = spec_.get_angular_speed() * dt;
  if (std::abs(delta) <= rot_step) {
    telemetry_.set_current_direction(command_.dir);
  }
  else {
    telemetry_.set_current_direction(telemetry_.get_current_direction() + std::copysign(rot_step, delta));
  }

  // calculate speed
  float speed = telemetry_.get_current_speed();
  switch (command_.state) {
    case DroneMode::ACCELERATING:
      speed = std::min(speed + spec_.get_acceleration() * dt, spec_.get_attack_speed());
      break;
    case DroneMode::DECELERATING:
      speed = std::max(speed - spec_.get_acceleration() * dt, 0.0F);
      break;
    case DroneMode::MOVING:
      speed = spec_.get_attack_speed();
      break;
    case DroneMode::TURNING:
    case DroneMode::STOPPED:
      speed = 0.0F;
      break;
  }
  telemetry_.set_current_speed(speed);

  // update coordinates
  telemetry_.set_position(telemetry_.get_position() +
                          Coord{std::cos(telemetry_.get_current_direction()), std::sin(telemetry_.get_current_direction())} * (speed * dt));

  // update clock
  telemetry_.set_elapsed(telemetry_.elapsed() + dt);
}

void DronePhysics::tick()
{
  if (channel_.size() > 0) {
    std::lock_guard<std::mutex> command_guard(command_mtx_);
    if (auto command = channel_.drain_to_last())
      command_ = *command;
  }
  {
    std::lock_guard<std::mutex> telemetry_guard(tel_mtx_);
    step(physics_time_step);
  }
}

void DronePhysics::submit_command(const DroneCommand& command) const
{
  channel_.emplace(command);
}

const DroneSpec DronePhysics::get_spec() const
{
  return spec_;
}

const IState* DronePhysics::get_state() const
{
  std::lock_guard<std::mutex> g(command_mtx_);
  return state_from_mode(command_.state);
}

const DroneTelemetry DronePhysics::get_telemetry() const
{
  std::lock_guard<std::mutex> telemetry_guard(tel_mtx_);
  return telemetry_;
}

const DroneCommand DronePhysics::get_active_command() const
{
  std::lock_guard<std::mutex> command_guard(command_mtx_);
  return command_;
}