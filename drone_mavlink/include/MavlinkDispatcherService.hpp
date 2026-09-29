#pragma once

#include "ThreadWorker.hpp"
#include "UdpLink.hpp"

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mavlink_types.h>

using MessageHandler = std::function<void(const mavlink_message_t&)>;
using MessageId = uint32_t;

// dispatching incoming udp messages by handlers
class MavlinkDispatcherService : public ThreadWorker {
  static constexpr int RX_POLL_MS = 100;

  UdpLink& link_;
  std::unordered_map<MessageId, MessageHandler> handlers_;

  void run_loop() override
  {
    std::vector<mavlink_message_t> batch;

    while (running_) {
      batch.clear();
      link_.receive(batch, RX_POLL_MS);

      for (const auto& msg : batch) {
        const auto it = handlers_.find(msg.msgid);
        if (it == handlers_.end()) {
          continue;  // unhandled by design: draining is the point
        }
        it->second(msg);
      }
    }
  }

public:
  explicit MavlinkDispatcherService(UdpLink& link)
    : link_(link)
  {
  }

  void register_handler(MessageId msgid, MessageHandler handler) { handlers_[msgid] = std::move(handler); }
};
