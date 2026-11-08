#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };

std::vector<std::pair<std::size_t, std::size_t>> matchTimestamps(const std::vector<double>& lidar,
                                                                 const std::vector<double>& cam, double max_dt) {
  std::vector<std::pair<std::size_t, std::size_t>> out;
  if (cam.empty()) return out;
  for (std::size_t i = 0; i < lidar.size(); ++i) {
    const double t = lidar[i];
    // First camera stamp >= t. The nearest is either it or the one just before it.
    auto it = std::lower_bound(cam.begin(), cam.end(), t);
    if (it != cam.begin()) {
      auto prev = std::prev(it);
      // `<=`: on a tie (or if it == end) prefer the earlier frame. With duplicate stamps,
      // move prev back to the FIRST equal value so the smallest index wins.
      if (it == cam.end() || t - *prev <= *it - t) it = std::lower_bound(cam.begin(), prev, *prev);
    }
    const double dt = std::abs(*it - t);
    if (dt <= max_dt) out.emplace_back(i, static_cast<std::size_t>(it - cam.begin()));
  }
  return out;  // O(n log m). Two-pointer sweep: O(n + m), since both inputs are sorted.
}

std::size_t partitionGround(std::vector<Point3>& cloud, double z_thresh) {
  // stable_partition keeps relative order inside both groups. O(n) with a temp buffer, O(n log n) without.
  auto mid = std::stable_partition(cloud.begin(), cloud.end(), [z_thresh](const Point3& p) { return p.z < z_thresh; });
  return static_cast<std::size_t>(mid - cloud.begin());
}
