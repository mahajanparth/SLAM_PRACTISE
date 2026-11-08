#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

struct Point3 { double x, y, z; };

Point3 parallelCentroid(const std::vector<Point3>& cloud, int num_threads) {
  // TODO: chunk, one std::thread per chunk, per-thread partial sums, join, combine
  return {0, 0, 0};
}

std::size_t countInRangeParallel(const std::vector<Point3>& cloud, double max_range, int num_threads) {
  // TODO: std::atomic<std::size_t> total
  return 0;
}

class HitCounter {
 public:
  void add(int cell, int n = 1) {
    // TODO
  }
  int get(int cell) const {
    // TODO
    return 0;
  }
  std::size_t distinct() const {
    // TODO
    return 0;
  }

 private:
  mutable std::mutex m_;
  std::unordered_map<int, int> counts_;
};
