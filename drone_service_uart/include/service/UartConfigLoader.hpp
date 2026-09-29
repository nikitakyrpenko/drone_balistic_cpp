#pragma once

#include "DroneLink.hpp"
#include "ScheduledWorker.hpp"
#include "UartLink.hpp"
#include "dto/AmmoDTO.hpp"
#include "dto/ConfigDTO.hpp"
#include "dto/TargetDTO.hpp"
#include "service/interfaces/IConfigLoader.hpp"

#include <latch>
#include <memory>

class UartConfigLoader : public IConfigLoader, public ScheduledWorker {
  std::shared_ptr<UartLink> link_;

  ConfigDTO config_;
  AmmoDTO arsenal_;

  std::latch ready_{3};
  bool ammo_done_{false};
  bool cfg_done_{false};
  bool telemetry_done_{false};

  void tick() override;

public:
  explicit UartConfigLoader(std::shared_ptr<UartLink> link);

  void wait_ready() const override;

  const ConfigDTO& get_config() const override;
  const AmmoDTO& get_arsenal() const override;
  const TargetDTO& get_targets() const override;
};
