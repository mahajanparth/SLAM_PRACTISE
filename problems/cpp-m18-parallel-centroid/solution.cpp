#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

struct Point3 { double x, y, z; };

// Clamp the requested thread count to [1, n] (n >= 1 assumed).
static std::size_t threadCount(int requested, std::size_t n) {
  const std::size_t r = requested < 1 ? 1 : static_cast<std::size_t>(requested);
  return std::min(r, n);
}

Point3 parallelCentroid(const std::vector<Point3>& cloud, int num_threads) {
  const std::size_t n = cloud.size();
  if (n == 0) return {0, 0, 0};
  const std::size_t T = threadCount(num_threads, n);
  std::vector<Point3> partial(T, Point3{0, 0, 0});  // one slot per thread: no shared writes
  std::vector<std::thread> workers;
  workers.reserve(T);
  for (std::size_t t = 0; t < T; ++t) {
    workers.emplace_back([&cloud, &partial, t, n, T] {  // t captured BY VALUE
      const std::size_t begin = t * n / T, end = (t + 1) * n / T;
      Point3 s{0, 0, 0};  // accumulate in a local (register), write once -> no false sharing in the loop
      for (std::size_t i = begin; i < end; ++i) { s.x += cloud[i].x; s.y += cloud[i].y; s.z += cloud[i].z; }
      partial[t] = s;
    });
  }
  for (auto& w : workers) w.join();  // join() also makes partial[] writes visible to this thread
  Point3 sum{0, 0, 0};
  for (const auto& p : partial) { sum.x += p.x; sum.y += p.y; sum.z += p.z; }
  const double inv = 1.0 / static_cast<double>(n);
  return {sum.x * inv, sum.y * inv, sum.z * inv};
}

std::size_t countInRangeParallel(const std::vector<Point3>& cloud, double max_range, int num_threads) {
  const std::size_t n = cloud.size();
  if (n == 0) return 0;
  const std::size_t T = threadCount(num_threads, n);
  const double r2 = max_range * max_range;
  std::atomic<std::size_t> total{0};
  std::vector<std::thread> workers;
  workers.reserve(T);
  for (std::size_t t = 0; t < T; ++t) {
    workers.emplace_back([&, t] {
      std::size_t local = 0;
      for (std::size_t i = t * n / T; i < (t + 1) * n / T; ++i) {
        const Point3& p = cloud[i];
        if (max_range >= 0 && p.x * p.x + p.y * p.y + p.z * p.z <= r2) ++local;
      }
      total.fetch_add(local, std::memory_order_relaxed);  // one atomic op per thread, not per point
    });
  }
  for (auto& w : workers) w.join();
  return total.load();
}

class HitCounter {
 public:
  void add(int cell, int n = 1) {
    std::lock_guard<std::mutex> lk(m_);
    counts_[cell] += n;
  }
  int get(int cell) const {
    std::lock_guard<std::mutex> lk(m_);  // readers lock too: a concurrent rehash would invalidate the lookup
    auto it = counts_.find(cell);
    return it == counts_.end() ? 0 : it->second;
  }
  std::size_t distinct() const {
    std::lock_guard<std::mutex> lk(m_);
    return counts_.size();
  }

 private:
  mutable std::mutex m_;  // mutable: locking is not a logical modification
  std::unordered_map<int, int> counts_;
};
