#include "UartPort.hpp"
#include "FdIo.hpp"

#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <cstddef>
#include <stdexcept>

UartPort::UartPort(const char* serial)
  : fd_([serial]() {
    int fd = open(serial, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
      throw std::runtime_error("cannot open serial");
    }
    return fd;
  }())
  , term_([this]() {
    termios tio{};

    if (tcgetattr(fd_, &tio) < 0) {
      throw std::runtime_error("failed to get serial attributes");
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);

    return tio;
  }())
{
  if (tcsetattr(fd_, TCSANOW, &term_) < 0) {
    throw std::runtime_error("failed to set serial attributes");
  }
}

FdIo::Result UartPort::write(const uint8_t* buf, size_t len, int timeout, size_t* bytes_written)
{
  return FdIo::write(fd_, buf, len, timeout, bytes_written);
}

FdIo::Result UartPort::read_some(uint8_t* buf, size_t limit, int timeout, size_t* bytes_read)
{
  return FdIo::read_some(fd_, buf, limit, timeout, bytes_read);
}

UartPort::~UartPort()
{
  close(fd_);
}
