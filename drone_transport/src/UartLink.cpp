#include "UartLink.hpp"
#include "FdIo.hpp"

#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

static constexpr size_t BUFF_LEN = 256;

UartLink::UartLink(const char* serial)
  : port_(std::make_unique<UartPort>(serial))
{
}

void UartLink::send(const dlink::Control& ctrl)
{
  uint8_t frame[sizeof(dlink::Control) + 6];
  const size_t len = dlink::encode(dlink::PKT_CONTROL, &ctrl, sizeof(ctrl), frame);

  size_t bytes_written = 0;
  const auto r = port_->write(frame, len, -1, &bytes_written);

  if (r != FdIo::Result::OK) {
    throw std::runtime_error("UartLink::send failed after " + std::to_string(bytes_written) + " of " + std::to_string(len) + " bytes");
  }
}

void UartLink::run_loop()
{
  dlink::Parser parser{};

  while (running_) {
    uint8_t buff[BUFF_LEN]{};

    size_t bytes_read = 0;
    const auto r = port_->read_some(buff, BUFF_LEN, -1, &bytes_read);

    uint8_t type, len, payload[260];

    if (r == FdIo::Result::FAILED) {
      std::cerr << "[UartLink] read failed, stopping link\n";
      interrupt();
      break;
    }

    for (size_t i = 0; i < bytes_read; i++) {
      if (parser.feed(buff[i], type, payload, len)) {
        switch (type) {
          case dlink::PacketType::PKT_TELEMETRY: {
            dlink::Telemetry t{};
            memcpy(&t, payload, sizeof(t));
            std::cout << "[UartLink] Telemetry pos: x=" << t.x << " y=" << t.y << " dir=" << t.dir << std::endl;
            telemetry_channel_.emplace(t);
            break;
          }
          case dlink::PacketType::PKT_AMMO: {
            dlink::AmmoCfg ammo{};
            memcpy(&ammo, payload, sizeof(ammo));
            ammo_channel_.emplace(ammo);
            break;
          }
          case dlink::PacketType::PKT_TARGET: {
            dlink::TargetPos target{};
            memcpy(&target, payload, sizeof(target));
            target_channel_.emplace(TimestampedTargetPos{target, std::chrono::steady_clock::now()});
            break;
          }
          case dlink::PacketType::PKT_RESULT: {
            dlink::Result result{};
            memcpy(&result, payload, sizeof(result));
            result_channel_.emplace(result);
            break;
          }
          case dlink::PacketType::PKT_CONFIG: {
            dlink::DroneCfg cfg{};
            memcpy(&cfg, payload, sizeof(cfg));
            config_channel_.emplace(cfg);
            break;
          }
          default:
            break;
        }
      }
    }

    if (r == FdIo::Result::CLOSED) {
      std::cerr << "[UartLink] serial port closed, stopping link\n";
      interrupt();
      break;
    }
  }
}
