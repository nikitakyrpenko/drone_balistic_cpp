#pragma once

#include "service/interfaces/IFirepointProvider.hpp"

class FirepointProvider : public IFirepointProvider {
public:
  const BallisticSolution solve(const DroneTelemetry& tel,
                                const DroneSpec& spec,
                                const FallResult& fall,
                                const Coord& target) const override;

  ~FirepointProvider() override;
};