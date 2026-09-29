#pragma once

#include "models/Coord.hpp"
#include "service/interfaces/IBallisticSolutionEvaluator.hpp"

class BallisticSolutionEvaluator : public IBallisticSolutionEvaluator {
  struct Turn {
    Coord position;   // position of zero velocity
    float time;       // decel_time + turn_time
    float direction;  // direction after turn
  };

  struct CompResult {
    float time;
    float direction;
  };

  float calculate_time_to_reach(
    const Coord& from, const Coord& to, float current_speed, float attack_speed, float acceleration, bool decelerate_in_dest) const;

  Turn calculate_time_to_turn(const Coord& from,
                              const Coord& to,
                              float current_direction,
                              float current_speed,
                              float turn_threshold,
                              float angular_speed,
                              float acceleration) const;

  CompResult calculate_leg(
    const DroneSpec& spec, const Coord& from, const Coord& to, float current_direction, float current_speed, bool decelerate_in_dest) const;

public:
  const Task calculate_time_taken(const DroneTelemetry& tel,
                                  const DroneSpec& spec,
                                  int target_id,
                                  const BallisticSolution& solution) const override;

  ~BallisticSolutionEvaluator() override{};
};
