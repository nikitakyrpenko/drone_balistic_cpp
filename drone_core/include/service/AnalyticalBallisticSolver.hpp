#pragma once

#include "service/interfaces/IBallisticSolver.hpp"

class AnalyticalBallisticSolver : public IBallisticSolver {
public:
  FallResult fall(const Ammo& ammo, float altitude, float speed) const override;
};
