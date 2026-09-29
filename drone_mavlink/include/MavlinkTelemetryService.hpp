#pragma once

#include "MavlinkConstants.hpp"
#include "ScheduledWorker.hpp"
#include "UdpLink.hpp"
#include "models/Coord.hpp"
#include "models/DroneTelemetry.hpp"
#include "service/interfaces/IDronePhysics.hpp"

#include <cmath>
#include <cstdint>
#include <memory>

using namespace mavlink_constants;

class MavlinkTelemetryService : public ScheduledWorker {
  static constexpr float PERIOD_S = 0.3F;

  UdpLink& link_;
  std::shared_ptr<const IDronePhysics> physics_;

  void tick() override
  {
    const DroneTelemetry telemetry = physics_->get_telemetry();
    const Coord position = telemetry.get_position();

    const float dir = telemetry.get_current_direction();
    const float speed = telemetry.get_current_speed();
    const float altitude = telemetry.get_altitude();

    const GlobalPosition global = to_global(position, altitude);

    const double hdg_deg = std::fmod(std::fmod(90.0 - dir * DEG_PER_RAD, 360.0) + 360.0, 360.0);

    const uint32_t time_boot_ms = static_cast<uint32_t>(telemetry.elapsed() * 1000.0F);

    link_.global_position(SYSTEM_ID,
                          COMPONENT_ID,
                          time_boot_ms,
                          global.lat_e7,
                          global.lon_e7,
                          static_cast<int32_t>(global.alt_m * 1000.0F),
                          static_cast<int32_t>(global.alt_m * 1000.0F),
                          static_cast<int16_t>(speed * std::sin(dir) * 100.0F),
                          static_cast<int16_t>(speed * std::cos(dir) * 100.0F),
                          0,
                          static_cast<uint16_t>(hdg_deg * 100.0));

    const float raw_yaw = std::numbers::pi_v<float> / 2.0F - dir;
    const float yaw = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));

    link_.attitude(SYSTEM_ID, COMPONENT_ID, time_boot_ms, 0.0F, 0.0F, yaw, 0.0F, 0.0F, 0.0F);
  }

public:
  MavlinkTelemetryService(UdpLink& link, std::shared_ptr<const IDronePhysics> physics)
    : ScheduledWorker(PERIOD_S)
    , link_(link)
    , physics_(std::move(physics))
  {
  }
};
