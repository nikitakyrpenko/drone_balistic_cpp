#pragma once

#include "models/DroneMode.hpp"

struct DroneCommand {
  DroneMode state{DroneMode::STOPPED};
  float dir{0.0F};
};
