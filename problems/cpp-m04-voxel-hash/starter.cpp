#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

struct Point3 { double x, y, z; };

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& o) const {
    // TODO
    return false;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    // TODO: mix all three coordinates
    return 0;
  }
};

VoxelKey toKey(const Point3& p, double voxel) {
  // TODO
  return VoxelKey{0, 0, 0};
}

std::vector<Point3> voxelDownsample(const std::vector<Point3>& cloud, double voxel) {
  // TODO
  return {};
}
