#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

struct ImuSample { double t; double gx, gy, gz; };

class ImuWindow {
 public:
  explicit ImuWindow(double window_sec) : window_(window_sec) {}

  bool push(const ImuSample& s) {
    if (!buf_.empty() && s.t < buf_.back().t) return false;  // out-of-order: reject
    buf_.push_back(s);                                       // O(1), never moves existing elements
    add(s, +1.0);
    const double cutoff = s.t - window_;
    while (buf_.front().t < cutoff) {                        // never empties: s itself survives
      add(buf_.front(), -1.0);
      buf_.pop_front();                                      // O(1), unlike vector::erase(begin())
    }
    return true;
  }

  std::size_t size() const { return buf_.size(); }

  std::pair<double, double> timeSpan() const {
    if (buf_.empty()) return {0.0, 0.0};
    return {buf_.front().t, buf_.back().t};
  }

  std::array<double, 3> meanGyro() const {
    if (buf_.empty()) return {0.0, 0.0, 0.0};
    const double n = static_cast<double>(buf_.size());
    return {sum_[0] / n, sum_[1] / n, sum_[2] / n};
  }

  std::vector<ImuSample> between(double t0, double t1) const {
    std::vector<ImuSample> out;
    if (t0 > t1) return out;
    // deque iterators are random access, so binary search works.
    auto it = std::lower_bound(buf_.begin(), buf_.end(), t0,
                               [](const ImuSample& a, double t) { return a.t < t; });
    for (; it != buf_.end() && it->t <= t1; ++it) out.push_back(*it);
    return out;
  }

 private:
  void add(const ImuSample& s, double sign) {
    sum_[0] += sign * s.gx; sum_[1] += sign * s.gy; sum_[2] += sign * s.gz;
  }

  double window_;
  std::deque<ImuSample> buf_;      // chunked storage: blocks of elements plus a map of block pointers
  std::array<double, 3> sum_{};    // running sums for O(1) mean
};
