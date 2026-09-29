#pragma once

#include "models/DroneSpec.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/Task.hpp"

class IBallisticSolutionEvaluator {
public:
  virtual const Task calculate_time_taken(const DroneTelemetry& tel,
                                          const DroneSpec& spec,
                                          int target_id,
                                          const BallisticSolution& solution) const = 0;

  virtual ~IBallisticSolutionEvaluator() = default;
};
