#pragma once
#include "models/Coord.hpp"

#include <vector>

struct TargetDTO {
  std::vector<std::vector<Coord>> positions_;
  int target_count_;
  int time_steps_;
};