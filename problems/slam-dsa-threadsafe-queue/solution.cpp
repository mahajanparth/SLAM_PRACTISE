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
    {
      std::lock_guard<std::mutex> lk(m_);
      if (shutdown_) return false;
      if (max_size_ > 0 && q_.size() >= max_size_) q_.pop_front();  // drop oldest
      q_.push_back(std::move(v));
    }
    cv_.notify_one();  // notifying outside the lock avoids waking a thread that immediately blocks on m_
    return true;
  }

  std::optional<T> pop() {
    std::unique_lock<std::mutex> lk(m_);
    // Predicate form re-checks after every (possibly spurious) wakeup.
    cv_.wait(lk, [&] { return shutdown_ || !q_.empty(); });
    if (q_.empty()) return std::nullopt;  // only reachable when shut down
    T v = std::move(q_.front());
    q_.pop_front();
    return v;
  }

  std::optional<T> tryPop() {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) return std::nullopt;
    T v = std::move(q_.front());
    q_.pop_front();
    return v;
  }

  void shutdown() {
    {
      std::lock_guard<std::mutex> lk(m_);
      shutdown_ = true;
    }
    cv_.notify_all();  // every blocked consumer must wake up and see the flag
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lk(m_);
    return q_.size();
  }

 private:
  std::size_t max_size_;
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::deque<T> q_;
  bool shutdown_ = false;
};
