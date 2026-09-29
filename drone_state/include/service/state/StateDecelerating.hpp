#pragma once

#include "service/interface/state/IState.hpp"

class StateDecelerating : public IState {
  static StateDecelerating instance;

public:
  const static IState* get_instance() { return &instance; }
  const StateDecision decide(const DroneSpec& spec, const DroneTelemetry& tel, const Coord& coord, bool) const override;
  inline std::string name() const override { return "DECELERATING"; };
  inline DroneMode mode() const override { return DroneMode::DECELERATING; };
};

inline StateDecelerating StateDecelerating::instance{};

#include "Calc.hpp"
#include "service/state/StateAccelerating.hpp"
#include "service/state/StateStopped.hpp"

#include <cmath>

inline const StateDecision StateDecelerating::decide(const DroneSpec& spec, const DroneTelemetry& tel, const Coord& coord, bool) const
{
  float delta = Calc::calculate_turning_angle(tel.get_position(), coord, tel.get_current_direction());
  float dir = Calc::angle(tel.get_position(), coord);

  if (tel.get_current_speed() <= 0.0F)
    return {StateStopped::get_instance(), dir};

  if (std::abs(delta) > spec.get_turn_threshold())
    return {StateDecelerating::get_instance(), dir};

  float remaining_deceleration_distance = (tel.get_current_speed() * tel.get_current_speed()) / (2.0F * spec.get_acceleration());
  float distance = Calc::lenght(tel.get_position(), coord);

  if (distance > remaining_deceleration_distance)
    return {StateAccelerating::get_instance(), dir};

  return {StateDecelerating::get_instance(), dir};
}
