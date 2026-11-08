#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

template <typename T>
class ThreadSafeQueue {
 public:
  explicit ThreadSafeQueue(std::size_t capacity = 0) : cap_(capacity) {}

  bool push(T v) {
    // TODO
    return false;
  }
  std::optional<T> pop() {
    // TODO: block until an item is available or the queue is closed
    return std::nullopt;
  }
  std::optional<T> tryPop() {
    // TODO
    return std::nullopt;
  }
  std::optional<T> popFor(std::chrono::milliseconds timeout) {
    // TODO
    return std::nullopt;
  }
  void close() {
    // TODO
  }
  std::size_t size() const {
    // TODO
    return 0;
  }
  bool closed() const {
    // TODO
    return false;
  }
  std::size_t dropped() const {
    // TODO
    return 0;
  }

 private:
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<T> q_;
  bool closed_ = false;
  std::size_t dropped_ = 0;
  std::size_t cap_;
};
