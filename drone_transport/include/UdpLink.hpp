#pragma once

#include "FdIo.hpp"
#include "UdpPort.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <vector>

#include <common/mavlink.h>
#include <mavlink_types.h>

class UdpLink {
  static constexpr int SEND_TIMEOUT_MS = 100;

  static constexpr size_t RX_BUF_LEN = 2048;

  UdpPort port_;
  std::mutex tx_mutex_;

  mavlink_message_t rx_msg_{};
  mavlink_status_t rx_status_{};

  bool send_locked(const mavlink_message_t& msg)
  {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    size_t written = 0;
    if (port_.write(buf, len, SEND_TIMEOUT_MS, &written) != FdIo::Result::OK) {
      std::cerr << "UdpLink: send failed\n";
      return false;
    }

    return true;
  }

public:
  UdpLink(const char* address, uint16_t port)
    : port_(address, port)
  {
  }

  UdpLink(const UdpLink&) = delete;
  UdpLink& operator=(const UdpLink&) = delete;

  bool heartbeat(uint8_t system_id,
                 uint8_t component_id,
                 uint8_t type,
                 uint8_t autopilot,
                 uint8_t base_mode,
                 uint32_t custom_mode,
                 uint8_t system_status)
  {
    const std::lock_guard<std::mutex> lock(tx_mutex_);

    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, base_mode, custom_mode, system_status);

    return send_locked(msg);
  }

  bool global_position(uint8_t system_id,
                       uint8_t component_id,
                       uint32_t time_boot_ms,
                       int32_t lat,
                       int32_t lon,
                       int32_t alt,
                       int32_t relative_alt,
                       int16_t vx,
                       int16_t vy,
                       int16_t vz,
                       uint16_t hdg)
  {
    const std::lock_guard<std::mutex> lock(tx_mutex_);

    mavlink_message_t msg{};
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg, time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg);

    return send_locked(msg);
  }

  bool attitude(uint8_t system_id,
                uint8_t component_id,
                uint32_t time_boot_ms,
                float roll,
                float pitch,
                float yaw,
                float rollspeed,
                float pitchspeed,
                float yawspeed)
  {
    const std::lock_guard<std::mutex> lock(tx_mutex_);

    mavlink_message_t msg{};
    mavlink_msg_attitude_pack(system_id, component_id, &msg, time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);

    return send_locked(msg);
  }

  bool command_long(uint8_t system_id,
                    uint8_t component_id,
                    uint8_t target_system,
                    uint8_t target_component,
                    uint16_t command,  // MAV_CMD_USER_1
                    uint8_t confirmation,
                    float param1,
                    float param2,
                    float param3,
                    float param4,
                    float param5,  // lat
                    float param6,  // long
                    float param7)  // alt
  {
    const std::lock_guard<std::mutex> lock(tx_mutex_);

    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(system_id,
                                  component_id,
                                  &msg,
                                  target_system,
                                  target_component,
                                  command,
                                  confirmation,
                                  param1,
                                  param2,
                                  param3,
                                  param4,
                                  param5,
                                  param6,
                                  param7);

    return send_locked(msg);
  }

  size_t receive(std::vector<mavlink_message_t>& out, int timeout)
  {
    uint8_t buf[RX_BUF_LEN];
    size_t bytes_read = 0;

    if (port_.read_some(buf, RX_BUF_LEN, timeout, &bytes_read) != FdIo::Result::OK) {
      return 0;
    }

    size_t decoded = 0;

    for (size_t i = 0; i < bytes_read; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx_msg_, &rx_status_) == MAVLINK_FRAMING_OK) {
        out.push_back(rx_msg_);
        decoded++;
      }
    }
    return decoded;
  }
};
