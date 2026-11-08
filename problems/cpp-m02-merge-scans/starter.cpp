#include <algorithm>
#include <cstddef>
#include <functional>
#include <queue>
#include <tuple>
#include <vector>

struct Point3 { double x, y, z; };
struct StampedPoint { double t; Point3 p; };

void appendScan(std::vector<Point3>& dst, const std::vector<Point3>& src) {
  // TODO: at most one reallocation, safe when &dst == &src
}

std::vector<StampedPoint> mergeSorted(const std::vector<std::vector<StampedPoint>>& scans) {
  // TODO: allocate once, merge by timestamp
  return {};
}
