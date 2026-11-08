#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

struct Point3 { double x, y, z; };

void removeOutOfRange(std::vector<Point3>& cloud, double min_r, double max_r) {
  // TODO: erase-remove idiom
}

void removeEveryNth(std::vector<Point3>& cloud, std::size_t n) {
  // TODO: in place, O(n)
}

std::size_t dedupConsecutive(std::vector<Point3>& cloud, double eps) {
  // TODO
  return 0;
}
