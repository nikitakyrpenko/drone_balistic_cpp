#pragma once

#include <chrono>

struct RetryPolicy {
  int max_attempts = 5;
  std::chrono::milliseconds backoff{1000};
  int connect_timeout_ms = 2000;
  int send_timeout_ms = 2000;
  int recv_timeout_ms = 2000;
};
