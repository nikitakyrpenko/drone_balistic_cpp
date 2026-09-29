#pragma once

#include "Calc.hpp"
#include "service/interface/state/IState.hpp"

#include <cmath>

class StateStopped : public IState {
  static StateStopped instance;

public:
  const static IState* get_instance() { return &instance; }
  const StateDecision decide(const DroneSpec& spec, const DroneTelemetry& tel, const Coord& coord, bool) const override;
  inline std::string name() const override { return "STOPPED"; };
  inline DroneMode mode() const override { return DroneMode::STOPPED; };
};

inline StateStopped StateStopped::instance{};

#include "service/state/StateAccelerating.hpp"
#include "service/state/StateTurning.hpp"

inline const StateDecision StateStopped::decide(const DroneSpec& spec, const DroneTelemetry& tel, const Coord& coord, bool) const
{
  float delta = Calc::calculate_turning_angle(tel.get_position(), coord, tel.get_current_direction());
  float dir = Calc::angle(tel.get_position(), coord);

  if (std::abs(delta) <= spec.get_turn_threshold()) {
    return {StateAccelerating::get_instance(), dir};
  }

  return {StateTurning::get_instance(), dir};
}

