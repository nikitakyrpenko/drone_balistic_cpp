#pragma once

#include "models/Target.hpp"

#include <vector>

class ITargetProvider {
public:
  virtual const Target get_target(int id) const = 0;
  virtual std::vector<Target> get_targets() const = 0;
  virtual int get_size() const = 0;

  virtual ~ITargetProvider() = default;
};