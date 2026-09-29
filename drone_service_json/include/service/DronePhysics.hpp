#pragma once

#include "ScheduledWorker.hpp"
#include "ThreadSafeQueue.hpp"

#include "service/interfaces/IDronePhysics.hpp"
#include "service/interfaces/IConfigLoader.hpp"

#include <cmath>
#include <mutex>

class DronePhysics : public IDronePhysics, public ScheduledWorker {
  mutable SynchronizedQueue<DroneCommand> channel_;

  const float physics_time_step{0.1F};

  DroneTelemetry telemetry_;
  const DroneSpec spec_;
  DroneCommand command_{};

  mutable std::mutex tel_mtx_;
  mutable std::mutex command_mtx_;

  void step(float dt);
  void tick() override;

public:
  explicit DronePhysics(const IConfigLoader& c)
    : ScheduledWorker(c.get_config().physics_timestep / c.get_config().timescale)
    , physics_time_step(c.get_config().physics_timestep)
    , telemetry_(c.get_config().position_, c.get_config().initial_direction_, c.get_config().altitude_)
    , spec_(c.get_config().attack_speed_, c.get_config().acceleration_path_, c.get_config().angular_speed_, c.get_config().turn_threshold_)
  {
  }

  void submit_command(const DroneCommand& command) const override;

  const DroneTelemetry get_telemetry() const override;
  const DroneSpec get_spec() const override;
  const DroneCommand get_active_command() const override;
  const IState* get_state() const override;
};
