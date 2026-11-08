#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

struct Point3 { double x, y, z; };

Point3 centroid(const std::vector<Point3>& c) {
  // TODO: std::accumulate
  return Point3{-1, -1, -1};
}

std::size_t countWithin(const std::vector<Point3>& c, const Point3& q, double r) {
  // TODO: std::count_if
  return 0;
}

void sortByRange(std::vector<Point3>& c) {
  // TODO
}

std::vector<std::size_t> kClosest(const std::vector<Point3>& c, const Point3& q, std::size_t k) {
  // TODO: iota + nth_element + sort
  return {};
}
