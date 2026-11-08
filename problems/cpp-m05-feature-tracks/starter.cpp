#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <set>
#include <tuple>
#include <unordered_set>
#include <vector>

struct Point3 { double x, y, z; };
struct FrameDiff {
  std::vector<int> tracked;
  std::vector<int> lost;
  std::vector<int> appeared;
};

FrameDiff diffFrames(const std::vector<int>& prev, const std::vector<int>& curr) {
  // TODO
  return {};
}

std::size_t countUniqueVoxels(const std::vector<Point3>& cloud, double voxel) {
  // TODO
  return 0;
}

bool hasDuplicateIds(const std::vector<int>& ids) {
  // TODO
  return true;
}
