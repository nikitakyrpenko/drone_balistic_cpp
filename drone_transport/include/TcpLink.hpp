#pragma once

#include <cstdint>
#include <optional>

#include "HttpRequest.hpp"
#include "HttpResponce.hpp"
#include "RetryPolicy.hpp"

class TcpLink {
  uint16_t port_;
  RetryPolicy policy_;

public:
  explicit TcpLink(uint16_t port = 80, RetryPolicy policy = {});

  std::optional<HttpResponse> dispatch(const HttpRequest& request) const;
};
