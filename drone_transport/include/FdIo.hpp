#pragma once

#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstddef>
#include <cstdint>

namespace FdIo {
enum class Result { OK, CLOSED, TIMEOUT, FAILED };

inline int remaining_ms(std::chrono::steady_clock::time_point deadline, int timeout)
{
  if (timeout < 0) {
    return -1;
  }
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
  return left > 0 ? static_cast<int>(left) : 0;
}

inline Result wait_ready(int fd, short events, int timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout < 0 ? 0 : timeout);

  for (;;) {
    pollfd pfd{fd, events, 0};
    const int r = poll(&pfd, 1, remaining_ms(deadline, timeout));
    if (r > 0) {
      if (pfd.revents & events) {
        return Result::OK;
      }
      if (pfd.revents & (POLLERR | POLLNVAL)) {
        return Result::FAILED;
      }
      if (pfd.revents & POLLHUP) {
        return Result::CLOSED;
      }
      return Result::FAILED;  // woke with none of the requested events
    }
    if (r == 0) {
      return Result::TIMEOUT;
    }
    if (errno == EINTR) {
      continue;
    }
    return Result::FAILED;
  }
}

inline Result write(int fd, const uint8_t* buf, size_t n, int timeout, size_t* out_written = nullptr)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout < 0 ? 0 : timeout);

  size_t bytes_written = 0;
  auto calculate = [&](Result r) {
    if (out_written != nullptr) {
      *out_written = bytes_written;
    }
    return r;
  };

  while (bytes_written < n) {
    const ssize_t result = ::write(fd, buf + bytes_written, n - bytes_written);
    if (result > 0) {
      bytes_written += static_cast<size_t>(result);
      continue;
    }
    if (result == 0) {
      return calculate(Result::FAILED);
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      const auto ready = wait_ready(fd, POLLOUT, remaining_ms(deadline, timeout));
      if (ready != Result::OK) {
        return calculate(ready);
      }
      continue;
    }
    if (errno == EPIPE || errno == ECONNRESET) {
      return calculate(Result::CLOSED);
    }
    return calculate(Result::FAILED);
  }
  return calculate(Result::OK);
}

enum class Stream { Byte, Datagram };

inline Result read_some(int fd, uint8_t* buf, size_t n, int timeout, size_t* out_read = nullptr, Stream kind = Stream::Byte)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout < 0 ? 0 : timeout);

  size_t bytes_read = 0;
  auto calculate = [&](Result r) {
    if (out_read != nullptr) {
      *out_read = bytes_read;
    }
    return r;
  };

  for (;;) {
    const ssize_t result = ::read(fd, buf, n);
    if (result > 0) {
      bytes_read = static_cast<size_t>(result);
      return calculate(Result::OK);
    }
    if (result == 0) {
      return calculate(kind == Stream::Datagram ? Result::OK : Result::CLOSED);
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      const auto ready = wait_ready(fd, POLLIN, remaining_ms(deadline, timeout));
      if (ready != Result::OK) {
        return calculate(ready);
      }
      continue;
    }
    if (errno == ECONNRESET || errno == ENXIO || errno == EIO) {
      return calculate(Result::CLOSED);
    }
    return calculate(Result::FAILED);
  }
}
}  // namespace FdIo
