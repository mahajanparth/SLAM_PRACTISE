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
  ScanBuilder& stamp(double t) {
    // TODO
    return *this;
  }
  ScanBuilder& frame(std::string id) {
    // TODO
    return *this;
  }
  ScanBuilder& addRange(double r) {
    // TODO
    return *this;
  }
  ScanBuilder& addRanges(const std::vector<double>& rs) {
    // TODO
    return *this;
  }
  const std::vector<double>& ranges() const { return ranges_; }

  LidarScan build() const& {
    // TODO: copy
    return LidarScan{};
  }
  LidarScan build() && {
    // TODO: move
    return LidarScan{};
  }

 private:
  double stamp_ = 0.0;
  std::string frame_;
  std::vector<double> ranges_;
};

std::vector<double> clampRanges(std::vector<double> r, double min_r = 0.1, double max_r = 100.0) {
  // TODO
  return {};
}
