
#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include "TcpLink.hpp"
#include "HttpRequest.hpp"
#include "HttpResponce.hpp"
#include "TcpPort.hpp"

namespace {
constexpr size_t CHUNK_SIZE = 1024;

size_t write_to_sock(const uint8_t* buf, size_t len, size_t chunk_size, TcpPort& port, int timeout)
{
  size_t sent = 0;

  while (sent < len) {
    const size_t n = std::min(chunk_size, len - sent);

    size_t written = 0;
    const auto r = port.write(buf + sent, n, timeout, &written);

    sent += written;

    if (r != FdIo::Result::OK) {
      return sent;
    }
  }

  return sent;
}

std::optional<std::string> read_from_sock(TcpPort& port, int timeout)
{
  std::string response;
  uint8_t buf[CHUNK_SIZE];

  for (;;) {
    size_t n = 0;
    const auto r = port.read_some(buf, sizeof(buf), timeout, &n);

    response.append(reinterpret_cast<const char*>(buf), n);

    if (r == FdIo::Result::CLOSED) {
      return response;
    }
    if (r != FdIo::Result::OK) {
      return std::nullopt;
    }
  }
}

std::string_view trim(std::string_view value)
{
  const size_t first = value.find_first_not_of(" \t");
  if (first == std::string_view::npos) {
    return {};
  }
  return value.substr(first, value.find_last_not_of(" \t") - first + 1);
}

std::optional<HttpResponse> parse_http_response(std::string&& response)
{
  HttpResponse res;
  res.raw = std::move(response);

  std::string_view rest{res.raw};

  const size_t status_end = rest.find("\r\n");
  if (status_end == std::string_view::npos) {
    return std::nullopt;
  }

  const std::string_view status_line = rest.substr(0, status_end);
  rest.remove_prefix(status_end + 2);

  const size_t first_space = status_line.find(' ');
  if (first_space == std::string_view::npos) {
    return std::nullopt;
  }

  res.version = status_line.substr(0, first_space);

  const size_t second_space = status_line.find(' ', first_space + 1);
  if (second_space == std::string_view::npos) {
    res.status_code = status_line.substr(first_space + 1);
  }
  else {
    res.status_code = status_line.substr(first_space + 1, second_space - first_space - 1);
    res.status_text = trim(status_line.substr(second_space + 1));
  }

  if (res.version.compare(0, 5, "HTTP/") != 0 || res.status_code.size() != 3 ||
      res.status_code.find_first_not_of("0123456789") != std::string_view::npos) {
    return std::nullopt;
  }

  for (;;) {
    const size_t line_end = rest.find("\r\n");
    if (line_end == std::string_view::npos) {
      return std::nullopt;
    }

    if (line_end == 0) {
      rest.remove_prefix(2);
      break;
    }

    const std::string_view header_line = rest.substr(0, line_end);
    const size_t colon = header_line.find(':');

    if (colon != std::string_view::npos) {
      res.headers.emplace(trim(header_line.substr(0, colon)), trim(header_line.substr(colon + 1)));
    }

    rest.remove_prefix(line_end + 2);
  }

  res.body = rest;
  return res;
}

int parse_status_code(std::string_view code)
{
  int status = 0;
  std::from_chars(code.data(), code.data() + code.size(), status);
  return status;
}

}  // namespace

TcpLink::TcpLink(uint16_t port, RetryPolicy policy)
  : port_(port)
  , policy_(policy)
{
}

std::optional<HttpResponse> TcpLink::dispatch(const HttpRequest& request) const
{
  std::optional<HttpResponse> response;

  for (int attempt = 0; attempt < policy_.max_attempts; attempt++) {
    if (attempt > 0) {
      std::cerr << "TcpLink goes sleep for: " << policy_.backoff.count() << " ms\n";
      std::this_thread::sleep_for(policy_.backoff);
    }

    try {
      TcpPort port(request.host().c_str(), port_, policy_.connect_timeout_ms);

      const std::string header = request.serialize_header();

      if (write_to_sock(reinterpret_cast<const uint8_t*>(header.data()), header.size(), CHUNK_SIZE, port, policy_.send_timeout_ms) !=
          header.size()) {
        std::cerr << "Attempt " << attempt + 1 << ": short header write\n";
        continue;
      }

      const std::string& body = request.body();

      if (write_to_sock(reinterpret_cast<const uint8_t*>(body.data()), body.size(), CHUNK_SIZE, port, policy_.send_timeout_ms) !=
          body.size()) {
        std::cerr << "Attempt " << attempt + 1 << ": short body write\n";
        continue;
      }

      auto raw = read_from_sock(port, policy_.recv_timeout_ms);
      if (!raw) {
        std::cerr << "Attempt " << attempt + 1 << ": response lost after full send\n";
        if (request.method() == HttpRequest::Method::Post) {
          std::cout << "No retry for POST after full send\n";
          return std::nullopt;
        }
        continue;
      }

      auto parsed = parse_http_response(std::move(*raw));
      if (!parsed) {
        std::cerr << "Attempt " << attempt + 1 << ": unparseable response\n";
        if (request.method() == HttpRequest::Method::Post) {
          return std::nullopt;
        }
        continue;
      }

      const int status = parse_status_code(parsed->status_code);
      response = std::move(parsed);

      if (status == 503) {
        std::cerr << "Attempt " << attempt + 1 << ": service unavailable (503)\n";
        continue;
      }

      return response;
    }
    catch (const std::exception& e) {
      std::cerr << "Attempt " << attempt + 1 << ": " << e.what() << "\n";
    }
  }

  return response;
}
