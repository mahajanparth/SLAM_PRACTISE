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
    // TODO
    return false;
  }
  std::size_t size() const {
    // TODO
    return 0;
  }
  std::pair<double, double> timeSpan() const {
    // TODO
    return {-1.0, -1.0};
  }
  std::array<double, 3> meanGyro() const {
    // TODO: O(1)
    return {-1.0, -1.0, -1.0};
  }
  std::vector<ImuSample> between(double t0, double t1) const {
    // TODO
    return {};
  }

 private:
  double window_;
  std::deque<ImuSample> buf_;
  std::array<double, 3> sum_{};
};
