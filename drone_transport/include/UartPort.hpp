#pragma once

#include <termios.h>
#include <unistd.h>
#include <cstddef>
#include <cstdint>
#include "FdIo.hpp"

class UartPort {
  const int fd_{};
  const termios term_{};

public:
  explicit UartPort(const char* serial);

  FdIo::Result write(const uint8_t* buf, size_t len, int timeout = -1, size_t* written = nullptr);
  FdIo::Result read_some(uint8_t* buf, size_t limit, int timeout = -1, size_t* read_bytes = nullptr);

  ~UartPort();
};
