#pragma once

#include "dto/AmmoDTO.hpp"
#include "dto/ConfigDTO.hpp"
#include "dto/TargetDTO.hpp"

class IConfigLoader {
public:
  virtual const ConfigDTO& get_config() const = 0;
  virtual const AmmoDTO& get_arsenal() const = 0;
  virtual const TargetDTO& get_targets() const = 0;

  // blocks others untill data is fetched, primary usage in uart implementation
  virtual void wait_ready() const {}

  virtual ~IConfigLoader() = default;
};
