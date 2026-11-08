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
    {
      std::lock_guard<std::mutex> lk(m_);
      if (closed_) return false;
      if (cap_ > 0 && q_.size() == cap_) {  // drop-oldest: the producer never blocks
        q_.pop_front();
        ++dropped_;
      }
      q_.push_back(std::move(v));
    }
    cv_.notify_one();  // one new item -> one consumer can make progress; notify outside the lock
    return true;
  }

  std::optional<T> pop() {
    std::unique_lock<std::mutex> lk(m_);
    // The predicate is re-checked after every wakeup (spurious wakeups, or another consumer won the race).
    cv_.wait(lk, [this] { return !q_.empty() || closed_; });
    return takeFrontLocked();
  }

  std::optional<T> tryPop() {
    std::lock_guard<std::mutex> lk(m_);
    return takeFrontLocked();
  }

  std::optional<T> popFor(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait_for(lk, timeout, [this] { return !q_.empty() || closed_; });
    return takeFrontLocked();  // still empty -> timed out (or closed and drained)
  }

  void close() {
    {
      std::lock_guard<std::mutex> lk(m_);  // the flag is predicate state: must change under the mutex
      closed_ = true;
    }
    cv_.notify_all();  // EVERY waiter must re-check and exit
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lk(m_);
    return q_.size();
  }
  bool closed() const {
    std::lock_guard<std::mutex> lk(m_);
    return closed_;
  }
  std::size_t dropped() const {
    std::lock_guard<std::mutex> lk(m_);
    return dropped_;
  }

 private:
  // Precondition: m_ is held. Returns by value: a reference into q_ would escape the lock.
  std::optional<T> takeFrontLocked() {
    if (q_.empty()) return std::nullopt;
    std::optional<T> out(std::move(q_.front()));
    q_.pop_front();
    return out;
  }

  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<T> q_;
  bool closed_ = false;
  std::size_t dropped_ = 0;
  std::size_t cap_;
};
