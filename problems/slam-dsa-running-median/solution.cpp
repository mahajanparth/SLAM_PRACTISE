#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

class RunningMedian {
 public:
  void add(double x) {
    if (lo_.empty() || x <= lo_.top()) lo_.push(x);
    else hi_.push(x);
    // Rebalance so that lo_ has the same size as hi_ or exactly one more.
    if (lo_.size() > hi_.size() + 1) { hi_.push(lo_.top()); lo_.pop(); }
    else if (hi_.size() > lo_.size()) { lo_.push(hi_.top()); hi_.pop(); }
  }

  double median() const {
    if (lo_.empty()) return std::numeric_limits<double>::quiet_NaN();
    if (lo_.size() > hi_.size()) return lo_.top();
    return 0.5 * (lo_.top() + hi_.top());
  }

  std::size_t size() const { return lo_.size() + hi_.size(); }

 private:
  std::priority_queue<double> lo_;                                             // max-heap: lower half
  std::priority_queue<double, std::vector<double>, std::greater<double>> hi_;  // min-heap: upper half
};
