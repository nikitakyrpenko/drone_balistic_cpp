#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

template <typename T>
class SynchronizedQueue {
  std::mutex mtx_;
  std::queue<T> queue_;
  std::atomic<size_t> counter_{0};

public:
  template <typename... Args>

  void emplace(Args&&... args)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.emplace(std::forward<Args>(args)...);

    counter_++;
  }

  std::optional<T> drain_to_last()
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (queue_.empty()) {
      return std::nullopt;
    }

    T last = std::move(queue_.back());
    std::queue<T>{}.swap(queue_);

    counter_ = 0;

    return last;
  }

  const std::vector<T> drain_all()
  {
    std::lock_guard<std::mutex> l(mtx_);

    std::vector<T> result;
    result.reserve(queue_.size());

    while (!queue_.empty()) {
      result.push_back(std::move(queue_.front()));
      queue_.pop();
    }
    counter_ = 0;
    return result;
  }

  int size() const { return counter_; }
};