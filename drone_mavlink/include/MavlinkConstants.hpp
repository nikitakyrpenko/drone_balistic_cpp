#pragma once

#include "models/Coord.hpp"

#include <cmath>
#include <cstdint>
#include <common/mavlink.h>
#include <numbers>

namespace mavlink_constants {

inline constexpr uint8_t SYSTEM_ID = 1;
inline constexpr uint8_t COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;

inline constexpr uint8_t TARGET_SYSTEM_ID = 255;
inline constexpr uint8_t TARGET_COMPONENT_ID = 190;

inline constexpr auto COMMAND = MAV_CMD_USER_1;

inline constexpr uint8_t VEHICLE_TYPE = MAV_TYPE_QUADROTOR;
inline constexpr uint8_t AUTOPILOT = MAV_AUTOPILOT_GENERIC;
inline constexpr uint8_t BASE_MODE = MAV_MODE_FLAG_SAFETY_ARMED;
inline constexpr uint32_t CUSTOM_MODE = 0;
inline constexpr uint8_t SYSTEM_STATUS = MAV_STATE_ACTIVE;

inline constexpr double BASE_LAT = 50.4501;
inline constexpr double BASE_LONG = 30.5234;
inline constexpr double METERS_PER_DEG_LAT = 111320.0;
inline constexpr double DEG_PER_RAD = 180.0 / std::numbers::pi;

struct GlobalPosition {
  int32_t lat_e7;
  int32_t lon_e7;
  float alt_m;
};

inline GlobalPosition to_global(const Coord& position, float altitude_m)
{
  const double meters_per_deg_lon = METERS_PER_DEG_LAT * std::cos(BASE_LAT / DEG_PER_RAD);

  const double lat = BASE_LAT + position.y_ / METERS_PER_DEG_LAT;
  const double lon = BASE_LONG + position.x_ / meters_per_deg_lon;

  return {
    .lat_e7 = static_cast<int32_t>(std::lround(lat * 1e7)), .lon_e7 = static_cast<int32_t>(std::lround(lon * 1e7)), .alt_m = altitude_m};
}
}  // namespace mavlink_constants
