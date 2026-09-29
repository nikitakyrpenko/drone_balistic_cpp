#include "UdpPort.hpp"
#include "FdIo.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

UdpPort::UdpPort(const char* address, uint16_t port)
{
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  if (inet_pton(AF_INET, address, &addr.sin_addr) != 1) {
    throw std::runtime_error(std::string("Invalid UDP address: ") + address);
  }

  sock_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);

  if (sock_fd_ < 0) {
    throw std::runtime_error("Cannot instantiate UDP socket");
  }

  if (connect(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    const int err = errno;
    clean();
    throw std::runtime_error(std::string("Cannot set UDP peer ") + address + ": " + std::strerror(err));
  }

  std::cout << "UDP peer set to " << address << ":" << port << "\n";
}

FdIo::Result UdpPort::write(const uint8_t* buff, size_t len, int timeout, size_t* written)
{
  if (buff == nullptr) {
    return FdIo::Result::FAILED;
  }

  return FdIo::write(sock_fd_, buff, len, timeout, written);
}

FdIo::Result UdpPort::read_some(uint8_t* buff, size_t limit, int timeout, size_t* read_bytes)
{
  if (buff == nullptr) {
    return FdIo::Result::FAILED;
  }

  return FdIo::read_some(sock_fd_, buff, limit, timeout, read_bytes, FdIo::Stream::Datagram);
}

void UdpPort::clean()
{
  if (sock_fd_ >= 0) {
    close(sock_fd_);
    sock_fd_ = -1;
  }
}

UdpPort::~UdpPort()
{
  clean();
}
