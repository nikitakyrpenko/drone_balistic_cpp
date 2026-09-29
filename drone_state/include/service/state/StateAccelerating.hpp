#pragma once

#include <optional>
#include "Calc.hpp"
#include "models/DroneTelemetry.hpp"
#include "service/interface/state/IState.hpp"

class StateAccelerating : public IState {
  static StateAccelerating instance;

public:
  const static IState* get_instance() { return &instance; }
  const StateDecision decide(const DroneSpec& spec, const DroneTelemetry& tel, const Coord& coord, bool) const override;
  inline std::string name() const override { return "ACCELERATING"; };
  inline DroneMode mode() const override { return DroneMode::ACCELERATING; };
};

inline StateAccelerating StateAccelerating::instance{};

#include "service/state/StateDecelerating.hpp"
#include "service/state/StateMoving.hpp"
#include <cmath>

inline const StateDecision StateAccelerating::decide(const DroneSpec& spec,
                                                     const DroneTelemetry& tel,
                                                     const Coord& coord,
                                                     bool decelerate_in_dest) const
{
  float delta = Calc::calculate_turning_angle(tel.get_position(), coord, tel.get_current_direction());
  float angle = Calc::angle(tel.get_position(), coord);

  if (std::abs(delta) > spec.get_turn_threshold())
    return {StateDecelerating::get_instance(), angle};

  if (decelerate_in_dest) {
    float remaining_deceleration_distance = (tel.get_current_speed() * tel.get_current_speed()) / (2.0F * spec.get_acceleration());
    float distance = Calc::lenght(tel.get_position(), coord);

    if (distance <= remaining_deceleration_distance)
      return {StateDecelerating::get_instance(), angle};
  }

  if (tel.get_current_speed() >= spec.get_attack_speed()) {
    return {StateMoving::get_instance(), angle};
  }

  return {StateAccelerating::get_instance(), angle};
}
