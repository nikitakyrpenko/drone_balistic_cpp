#pragma once

#include "models/DroneTelemetry.hpp"
#include "models/StateDecision.hpp"
#include "models/DroneSpec.hpp"
#include "models/DroneMode.hpp"
#include "models/Coord.hpp"

#include <string>

class IState {
public:
  virtual const StateDecision decide(const DroneSpec& spec,
                                     const DroneTelemetry& tel,
                                     const Coord& coord,
                                     bool decelerate_in_dest = false) const = 0;
  virtual std::string name() const = 0;
  virtual DroneMode mode() const = 0;

  virtual ~IState() = default;
};