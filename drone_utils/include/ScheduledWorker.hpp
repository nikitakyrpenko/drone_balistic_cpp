#pragma once

#include "ThreadWorker.hpp"

#include <chrono>
#include <thread>

class ScheduledWorker : public ThreadWorker {
protected:
  virtual void tick() = 0;
  float idle_;

  void run_loop() override
  {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<float>(idle_));

    auto next = clock::now() + period;
    while (running_) {
      tick();
      std::this_thread::sleep_until(next);
      next += period;
    }
  }

public:
  explicit ScheduledWorker(float idle)
    : idle_(idle)
  {
  }
};
