#pragma once

#include <string>

class Ammo {
  const std::string name_{};
  const float mass_{};
  const float drag_{};
  const float lift_{};

public:
  Ammo(const std::string& name, float mass, float drag, float lift)
    : name_(name)
    , mass_(mass)
    , drag_(drag)
    , lift_(lift)
  {
  }

  const std::string& get_name() const { return name_; }
  float get_mass() const { return mass_; }
  float get_drag() const { return drag_; }
  float get_lift() const { return lift_; }
};
