#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

struct VoxelKey {
  int64_t x, y, z;
  bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

// Classic spatial hash (Teschner et al. 2003): multiply by large primes, then XOR.
struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    return static_cast<std::size_t>((k.x * 73856093LL) ^ (k.y * 19349663LL) ^ (k.z * 83492791LL));
  }
};

std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& pts, double leaf) {
  if (!(leaf > 0.0)) throw std::invalid_argument("leaf must be > 0");  // also rejects NaN
  struct Acc { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); int n = 0; };
  std::unordered_map<VoxelKey, Acc, VoxelKeyHash> grid;
  grid.reserve(pts.size());
  for (const auto& p : pts) {
    // floor, not a cast: int(-0.3) == 0 would merge voxels -1 and 0.
    const VoxelKey k{static_cast<int64_t>(std::floor(p.x() / leaf)), static_cast<int64_t>(std::floor(p.y() / leaf)),
                     static_cast<int64_t>(std::floor(p.z() / leaf))};
    Acc& a = grid[k];
    a.sum += p;
    ++a.n;
  }
  std::vector<Eigen::Vector3d> out;
  out.reserve(grid.size());
  for (const auto& kv : grid) out.push_back(kv.second.sum / kv.second.n);
  return out;
}
