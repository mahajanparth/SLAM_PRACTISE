#include <algorithm>
#include <cstddef>
#include <functional>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };
struct Stamped { double t; int sensor; };

std::vector<int> kNearest(const std::vector<Point3>& cloud, const Point3& q, int k) {
  // TODO: bounded max-heap
  return {};
}

std::vector<Stamped> mergeStreams(const std::vector<std::vector<Stamped>>& streams) {
  // TODO: min-heap k-way merge
  return {};
}
