#pragma once

#include "models/Ammo.hpp"

struct FallResult {
  float time;
  float distance;
};

class IBallisticSolver {
public:
  virtual FallResult fall(const Ammo& ammo, float altitude, float speed) const = 0;
  virtual ~IBallisticSolver() = default;
};
