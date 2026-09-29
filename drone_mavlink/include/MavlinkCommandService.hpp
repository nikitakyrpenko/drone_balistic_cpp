#pragma once

#include <common/mavlink.h>
#include <common/mavlink_msg_command_ack.h>
#include <mavlink_types.h>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>

#include "MavlinkConstants.hpp"
#include "UdpLink.hpp"
#include "models/Coord.hpp"

using namespace mavlink_constants;

class MavlinkCommandService {
  static constexpr uint8_t MAX_ATTEMPTS = 5;
  static constexpr int ACK_TIMEOUT_MS = 100;

  UdpLink& link_;

  std::mutex mtx_;
  std::condition_variable cv_;
  std::optional<mavlink_command_ack_t> ack_;
  std::optional<uint16_t> awaited_command_;

public:
  explicit MavlinkCommandService(UdpLink& link)
    : link_(link)
  {
  }

  void handle_ack(const mavlink_message_t& msg)
  {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    if (ack.result == MAV_RESULT_IN_PROGRESS) {
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(mtx_);
      if (!awaited_command_ || *awaited_command_ != ack.command) {
        return;
      }
      ack_ = ack;
    }
    cv_.notify_one();
  }

  bool command_long(const Coord& position, float altitude_m)
  {
    const auto global_pos = to_global(position, altitude_m);

    {
      const std::lock_guard<std::mutex> lock(mtx_);
      ack_.reset();
      awaited_command_ = COMMAND;
    }

    bool acked = false;

    for (uint8_t attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
      const bool is_ok = link_.command_long(SYSTEM_ID,
                                            COMPONENT_ID,
                                            TARGET_SYSTEM_ID,
                                            TARGET_COMPONENT_ID,
                                            COMMAND,
                                            attempt,
                                            0.F,
                                            0.F,
                                            0.F,
                                            0.F,
                                            static_cast<float>(global_pos.lat_e7 / 1e7),
                                            static_cast<float>(global_pos.lon_e7 / 1e7),
                                            global_pos.alt_m);

      if (!is_ok) {
        std::cout << "cannot send command long\n";
        break;
      }

      std::unique_lock<std::mutex> lock(mtx_);
      acked = cv_.wait_for(lock, std::chrono::milliseconds(ACK_TIMEOUT_MS), [this] { return ack_.has_value(); });
      if (acked) {
        break;
      }

      std::cout << "no COMMAND_ACK for " << COMMAND << ", attempt " << static_cast<int>(attempt) << "\n";
    }

    {
      const std::lock_guard<std::mutex> lock(mtx_);
      awaited_command_.reset();
    }

    return acked;
  }
};