#pragma once

#include "service/interface/state/IState.hpp"

class StateMoving : public IState {
  static StateMoving instance;

public:
  const static IState* get_instance() { return &instance; }
  const StateDecision decide(const DroneSpec& spec, const DroneTelemetry& tel, const Coord& coord, bool) const override;
  inline std::string name() const override { return "MOVING"; };
  inline DroneMode mode() const override { return DroneMode::MOVING; };
};

inline StateMoving StateMoving::instance{};

#include "Calc.hpp"
#include "service/state/StateDecelerating.hpp"
#include <cmath>

inline const StateDecision StateMoving::decide(const DroneSpec& spec,
                                               const DroneTelemetry& tel,
                                               const Coord& coord,
                                               bool decelerate_in_dest) const
{
  float delta = Calc::calculate_turning_angle(tel.get_position(), coord, tel.get_current_direction());
  float dir = Calc::angle(tel.get_position(), coord);

  if (std::abs(delta) > spec.get_turn_threshold()) {
    return {StateDecelerating::get_instance(), dir};
  }

  if (decelerate_in_dest) {
    float distance = Calc::lenght(tel.get_position(), coord);

    if (distance <= spec.get_acceleration_path()) {
      return {StateDecelerating::get_instance(), dir};
    }
  }

  return {StateMoving::get_instance(), dir};
}
