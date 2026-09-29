#pragma once

#include "models/DroneSpec.hpp"
#include "models/DroneTelemetry.hpp"
#include "models/Task.hpp"
#include "service/interfaces/IBallisticSolver.hpp"

class IFirepointProvider {
public:
  virtual const BallisticSolution solve(const DroneTelemetry& tel,
                                        const DroneSpec& spec,
                                        const FallResult& fall,
                                        const Coord& target) const = 0;
  virtual ~IFirepointProvider() = default;
};