#include "TcpPort.hpp"
#include "FdIo.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <poll.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

static constexpr int SYN_ACK_RETRIES = 3;

TcpPort::TcpPort(const char* domain_name, uint16_t port, int connect_timeout_ms)
{
  // DNS lookup to IPV4 or IPV6
  addrinfo hint{};
  hint.ai_family = AF_UNSPEC;  // IPv4 or IPv6
  hint.ai_socktype = SOCK_STREAM;

  addrinfo* addr_info = nullptr;

  int st = getaddrinfo(domain_name, std::to_string(port).c_str(), &hint, &addr_info);

  if (st != 0) {
    throw std::runtime_error("Cannot obtain ipaddr by DNS");
  }

  std::memcpy(&addr_, addr_info->ai_addr, addr_info->ai_addrlen);
  addr_len_ = addr_info->ai_addrlen;

  freeaddrinfo(addr_info);

  static const bool sigpipe_ignored = []() {
    std::signal(SIGPIPE, SIG_IGN);
    return true;
  }();
  (void)sigpipe_ignored;

  sock_fd_ = socket(addr_.ss_family, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP);

  if (sock_fd_ == -1)
    throw std::runtime_error("Cannot instantiate socket connection");

  // Non-fatal if unsupported; the poll timeout below is the real bound.
  (void)setsockopt(sock_fd_, IPPROTO_TCP, TCP_SYNCNT, &SYN_ACK_RETRIES, sizeof(SYN_ACK_RETRIES));

  const int connect_timeout = connect_timeout_ms > 0 ? connect_timeout_ms : -1;

  // initiate handshake
  const auto sync_r = connect(sock_fd_, reinterpret_cast<sockaddr*>(&addr_), addr_len_);

  if (sync_r < 0 && (errno != EINPROGRESS)) {
    std::cout << "Cannot establish connection with " << get_ip_addr() << "\n";
    clean();
    throw std::runtime_error("Cannot establish connection");
  }

  // poll sync ack
  if (sync_r < 0) {
    const auto syn_ack = FdIo::wait_ready(sock_fd_, POLLOUT, connect_timeout);
    if (syn_ack != FdIo::Result::OK) {
      std::cout << "Cannot receive SYN-ACK from " << get_ip_addr() << "\n";
      clean();
      throw std::runtime_error(syn_ack == FdIo::Result::TIMEOUT ? "Connection timed out" : "Cannot establish connection");
    }
  }

  // check is connection rejected
  int so_error = 0;
  socklen_t so_len = sizeof(so_error);
  if (getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, &so_error, &so_len) != 0 || so_error != 0) {
    std::cout << "Connection rejected by " << get_ip_addr() << "\n";
    clean();
    throw std::runtime_error("Cannot establish connection");
  }

  std::cout << "Established connection with " << get_ip_addr() << "\n";
}

FdIo::Result TcpPort::write(const uint8_t* buff, size_t len, int timeout, size_t* written)
{
  if (buff == nullptr) {
    return FdIo::Result::FAILED;
  }

  return FdIo::write(sock_fd_, buff, len, timeout, written);
}

FdIo::Result TcpPort::read_some(uint8_t* buff, size_t limit, int timeout, size_t* read_bytes)
{
  if (buff == nullptr) {
    return FdIo::Result::FAILED;
  }

  return FdIo::read_some(sock_fd_, buff, limit, timeout, read_bytes);
}

const char* TcpPort::get_ip_addr() const
{
  const char* out = nullptr;

  if (addr_.ss_family == AF_INET) {
    const sockaddr_in* v4 = reinterpret_cast<const sockaddr_in*>(&addr_);
    out = inet_ntop(AF_INET, &v4->sin_addr, ip_str_, sizeof(ip_str_));
  }
  else {
    const sockaddr_in6* v6 = reinterpret_cast<const sockaddr_in6*>(&addr_);
    out = inet_ntop(AF_INET6, &v6->sin6_addr, ip_str_, sizeof(ip_str_));
  }

  // Every caller is a diagnostic path; streaming a null pointer would be UB.
  return out != nullptr ? out : "<unknown>";
}

void TcpPort::clean()
{
  if (sock_fd_ >= 0) {
    close(sock_fd_);
    sock_fd_ = -1;
  }
}

TcpPort::~TcpPort()
{
  clean();
}