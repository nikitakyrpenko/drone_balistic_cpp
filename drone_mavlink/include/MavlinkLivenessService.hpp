#pragma once

#include "MavlinkConstants.hpp"
#include "ScheduledWorker.hpp"
#include "UdpLink.hpp"

using namespace mavlink_constants;

class MavlinkLivenessService : public ScheduledWorker {
  static constexpr float PERIOD_S = 1.0F;
  UdpLink& link_;

  void tick() override { link_.heartbeat(SYSTEM_ID, COMPONENT_ID, VEHICLE_TYPE, AUTOPILOT, BASE_MODE, CUSTOM_MODE, SYSTEM_STATUS); }

public:
  explicit MavlinkLivenessService(UdpLink& link)
    : ScheduledWorker(PERIOD_S)
    , link_(link)
  {
  }
};
