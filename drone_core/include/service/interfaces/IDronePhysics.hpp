#pragma once

#include "models/DroneTelemetry.hpp"
#include "models/DroneSpec.hpp"
#include "models/DroneCommand.hpp"
#include "service/interface/state/IState.hpp"

class IDronePhysics {
public:
  virtual void submit_command(const DroneCommand& command) const = 0;

  virtual const DroneTelemetry get_telemetry() const = 0;
  virtual const DroneSpec get_spec() const = 0;
  virtual const DroneCommand get_active_command() const = 0;
  virtual const IState* get_state() const = 0;

  virtual ~IDronePhysics() = default;
};