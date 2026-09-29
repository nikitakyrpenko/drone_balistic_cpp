#pragma once

#include <cstddef>
#include <cstdint>
#include "FdIo.hpp"

class UdpPort {
  int sock_fd_ = -1;

  void clean();

public:
  UdpPort(const char* address, uint16_t port);
  ~UdpPort();

  UdpPort(const UdpPort&) = delete;
  UdpPort& operator=(const UdpPort&) = delete;

  FdIo::Result write(const uint8_t* buff, size_t len, int timeout, size_t* written = nullptr);
  FdIo::Result read_some(uint8_t* buff, size_t limit, int timeout, size_t* read_bytes = nullptr);
};
