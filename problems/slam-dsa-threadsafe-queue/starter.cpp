#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

template <typename T>
class ThreadSafeQueue {
 public:
  explicit ThreadSafeQueue(std::size_t max_size = 0) : max_size_(max_size) {}

  bool push(T v) {
    // TODO
    return false;
  }
  std::optional<T> pop() {
    // TODO: block until item or shutdown (stub returns immediately)
    return std::nullopt;
  }
  std::optional<T> tryPop() {
    // TODO
    return std::nullopt;
  }
  void shutdown() {
    // TODO
  }
  std::size_t size() const {
    // TODO
    return 0;
  }

 private:
  std::size_t max_size_;
  // TODO: mutex, condition variable, container, shutdown flag
};
