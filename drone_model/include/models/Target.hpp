#pragma once

#include "Coord.hpp"

struct Target {
  int target_id_;
  Coord pos_;
  Coord vel_;

  Coord approximate(float dt) const { return pos_ + (vel_ * dt); }
};
