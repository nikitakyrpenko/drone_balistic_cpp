#pragma once

#include "models/Coord.hpp"

#include <string>

struct ConfigDTO {
  Coord position_;
  float altitude_{100.F};
  float initial_direction_;
  float attack_speed_;
  float acceleration_path_;
  float angular_speed_;
  float turn_threshold_;
  std::string ammo_;
  float target_array_timestep_;
  float time_step_;
  float physics_timestep{0.1F};
  float target_timestep{0.2F};
  float timescale;
  float hit_radius_;
};