#include <cstddef>
#include <deque>
#include <iterator>
#include <set>
#include <vector>

// Two balanced multisets: lo = smaller half (its max is the lower median), hi = larger half.
// Invariant: every element of lo <= every element of hi, and lo.size() == hi.size() or hi.size() + 1.
class RangeMedianFilter {
 public:
  explicit RangeMedianFilter(std::size_t k) : k_(k) {}

  double push(double r) {
    window_.push_back(r);
    if (lo_.empty() || r <= *lo_.rbegin()) lo_.insert(r); else hi_.insert(r);
    if (window_.size() > k_) {
      const double old = window_.front();
      window_.pop_front();
      // erase(find(v)) removes ONE copy. erase(v) would remove all duplicates.
      if (old <= *lo_.rbegin()) lo_.erase(lo_.find(old));  // old is in lo (ties live in lo first)
      else hi_.erase(hi_.find(old));
    }
    rebalance();
    if (lo_.size() > hi_.size()) return *lo_.rbegin();
    return 0.5 * (*lo_.rbegin() + *hi_.begin());
  }

  std::size_t size() const { return window_.size(); }

 private:
  void rebalance() {
    // Move the boundary element across: O(log k), and all other iterators stay valid.
    if (lo_.size() > hi_.size() + 1) {
      auto it = std::prev(lo_.end());
      hi_.insert(*it);
      lo_.erase(it);
    } else if (hi_.size() > lo_.size()) {
      auto it = hi_.begin();
      lo_.insert(*it);
      hi_.erase(it);
    }
  }

  std::size_t k_;
  std::deque<double> window_;   // arrival order, for eviction
  std::multiset<double> lo_, hi_;
};

// Why is "old <= max(lo)" enough to locate `old`? If old == max(lo) it is certainly in lo. If old < max(lo),
// it can't be in hi, since every hi element is >= max(lo). If old > max(lo), it must be in hi.

std::vector<double> medianFilter(const std::vector<double>& ranges, std::size_t k) {
  RangeMedianFilter f(k);
  std::vector<double> out;
  out.reserve(ranges.size());
  for (double r : ranges) out.push_back(f.push(r));
  return out;
}
