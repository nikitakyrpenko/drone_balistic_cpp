#pragma once

#include "models/Coord.hpp"
#include "models/Target.hpp"

#include <string>
#include <vector>

struct SimulationStep {
  int target_id_;
  float direction_;
  std::string state_;
  Coord position_;
  Coord drop_point_;
  Coord aim_point_;
  Coord predicted_target_;
  Coord target_position_;
  float elapsed_;
  float current_speed_;
  float ammo_mass_;
  float ammo_drag_;
  float ammo_lift_;
  float fall_time_;
  float fall_distance_;
  std::vector<Target> all_targets_;
};