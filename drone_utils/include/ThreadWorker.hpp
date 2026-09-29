#pragma once

#include <atomic>
#include <latch>
#include <thread>

class ThreadWorker {
  std::thread worker_;

protected:
  std::atomic<bool> running_{false};

  virtual void run_loop() = 0;

public:
  void start(std::latch& latch)
  {
    running_ = true;
    worker_ = std::thread([this, &latch]() {
      latch.arrive_and_wait();
      run_loop();
    });
  }

  void interrupt() { running_ = false; }

  virtual ~ThreadWorker()
  {
    interrupt();
    if (worker_.joinable())
      worker_.join();
  }
};
