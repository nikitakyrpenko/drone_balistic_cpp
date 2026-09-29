#pragma once

#include <string>
#include <vector>

struct AmmoDTO {
  struct Ammo {
    std::string name{};
    float mass{};
    float drag{};
    float lift{};
  };

  std::vector<Ammo> ammos_;
};