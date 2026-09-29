#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstddef>
#include <cstdint>
#include "FdIo.hpp"

class TcpPort {
  int sock_fd_ = -1;
  sockaddr_storage addr_{};
  socklen_t addr_len_{};
  mutable char ip_str_[INET6_ADDRSTRLEN]{};

  void clean();

public:
  TcpPort(const char* domain_name, uint16_t port = 80, int connect_timeout_ms = 0);
  ~TcpPort();

  TcpPort(const TcpPort&) = delete;
  TcpPort& operator=(const TcpPort&) = delete;

  FdIo::Result write(const uint8_t* buff, size_t len, int timeout, size_t* written = nullptr);
  FdIo::Result read_some(uint8_t* buff, size_t limit, int timeout, size_t* read_bytes = nullptr);

  const char* get_ip_addr() const;
};