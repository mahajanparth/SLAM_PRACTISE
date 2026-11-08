#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

struct Point3 { double x, y, z; };

void sortByDistance(std::vector<Point3>& cloud, const Point3& origin) {
  // Capture origin by reference: the comparator does not outlive this call.
  auto d2 = [&origin](const Point3& p) {
    const double dx = p.x - origin.x, dy = p.y - origin.y, dz = p.z - origin.z;
    return dx * dx + dy * dy + dz * dz;  // squared distance is monotonic in distance
  };
  // stable_sort keeps equal-distance points in their original order.
  std::stable_sort(cloud.begin(), cloud.end(),
                   [&d2](const Point3& a, const Point3& b) { return d2(a) < d2(b); });
}

std::size_t removeOutOfRange(std::vector<Point3>& cloud, double min_r, double max_r) {
  const std::size_t before = cloud.size();
  const double lo2 = min_r * min_r, hi2 = max_r * max_r;  // assumes 0 <= min_r
  auto outside = [lo2, hi2](const Point3& p) {
    const double r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    return r2 < lo2 || r2 > hi2;
  };
  cloud.erase(std::remove_if(cloud.begin(), cloud.end(), outside), cloud.end());
  return before - cloud.size();
}

auto makeBoxFilter(Point3 lo, Point3 hi) {
  // BY VALUE: lo/hi are parameters and die when we return; the closure keeps copies.
  return [lo, hi](const Point3& p) {
    return lo.x <= p.x && p.x <= hi.x && lo.y <= p.y && p.y <= hi.y && lo.z <= p.z &&
           p.z <= hi.z;
  };
}

auto makeIdGenerator(int first) {
  // Init-capture + mutable: the closure owns its counter; copying the closure copies the state.
  return [next = first]() mutable { return next++; };
}
