#include <algorithm>
#include <string>
#include <utility>
#include <vector>

struct LidarScan {
  double stamp = 0.0;
  std::string frame_id;
  std::vector<double> ranges;
};

class ScanBuilder {
 public:
  ScanBuilder& stamp(double t) { stamp_ = t; return *this; }

  // Sink parameter: one copy for lvalue callers (into `id`), zero for rvalue callers.
  ScanBuilder& frame(std::string id) { frame_ = std::move(id); return *this; }

  ScanBuilder& addRange(double r) { ranges_.push_back(r); return *this; }

  ScanBuilder& addRanges(const std::vector<double>& rs) {
    // Safe even when rs aliases ranges_: insert with a range from the same vector is UB, so copy first.
    if (&rs == &ranges_) { std::vector<double> tmp = rs; ranges_.insert(ranges_.end(), tmp.begin(), tmp.end()); }
    else ranges_.insert(ranges_.end(), rs.begin(), rs.end());
    return *this;
  }

  const std::vector<double>& ranges() const { return ranges_; }

  // Called on lvalues: the builder must stay intact, so copy.
  LidarScan build() const& { return LidarScan{stamp_, frame_, ranges_}; }

  // Called on rvalues: nobody will look at *this again, so steal the buffers.
  // The returned prvalue initialises the caller's object directly (guaranteed elision).
  LidarScan build() && { return LidarScan{stamp_, std::move(frame_), std::move(ranges_)}; }

 private:
  double stamp_ = 0.0;
  std::string frame_;
  std::vector<double> ranges_;
};

// By-value parameter: the caller decides whether to copy or move in.
// `return r;` implicitly moves a by-value parameter, so the buffer travels caller -> r -> result.
std::vector<double> clampRanges(std::vector<double> r, double min_r = 0.1, double max_r = 100.0) {
  for (double& x : r) x = std::clamp(x, min_r, max_r);
  return r;  // NOT std::move(r) (that's harmless here but noisy) and NOT a copy
}
