#pragma once

#include "service/interfaces/IConfigLoader.hpp"

#include <memory>

class ConfigLoader : public IConfigLoader {
  std::unique_ptr<ConfigDTO> config_;
  std::unique_ptr<AmmoDTO> arsenal_;
  std::unique_ptr<TargetDTO> targets_;

public:
  ConfigLoader(const char* config_source, const char* ammo_source, const char* target_source);

  const ConfigDTO& get_config() const override { return *config_; }
  const AmmoDTO& get_arsenal() const override { return *arsenal_; }
  const TargetDTO& get_targets() const override { return *targets_; }

  ~ConfigLoader() override;
};
