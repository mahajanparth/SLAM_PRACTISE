#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

struct Point3 { double x, y, z; };

void sortByDistance(std::vector<Point3>& cloud, const Point3& origin) {
  // TODO
}

std::size_t removeOutOfRange(std::vector<Point3>& cloud, double min_r, double max_r) {
  // TODO
  return 0;
}

auto makeBoxFilter(Point3 lo, Point3 hi) {
  // TODO: return a lambda bool(const Point3&)
  return [](const Point3&) { return false; };
}

auto makeIdGenerator(int first) {
  // TODO: return a lambda int()
  return []() { return 0; };
}
