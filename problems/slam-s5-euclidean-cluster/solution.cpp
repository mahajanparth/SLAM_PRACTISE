#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

class UnionFind {
 public:
  explicit UnionFind(int n) : parent_(n), size_(n, 1) { std::iota(parent_.begin(), parent_.end(), 0); }

  int find(int x) {
    // Iterative path halving: every node on the path points to its grandparent.
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];
      x = parent_[x];
    }
    return x;
  }

  bool unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b) return false;
    if (size_[a] < size_[b]) std::swap(a, b);  // union by size: attach smaller tree under larger
    parent_[b] = a;
    size_[a] += size_[b];
    return true;
  }

  int size(int x) { return size_[find(x)]; }

 private:
  std::vector<int> parent_, size_;
};

namespace {
struct VoxelHash {
  std::size_t operator()(const Eigen::Vector3i& v) const {
    return static_cast<std::size_t>(v.x()) * 73856093u ^ static_cast<std::size_t>(v.y()) * 19349663u ^
           static_cast<std::size_t>(v.z()) * 83492791u;
  }
};
struct VoxelEq {
  bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const { return a == b; }
};
}  // namespace

std::vector<int> euclideanCluster(const std::vector<Eigen::Vector3d>& pts, double r,
                                  int min_size = 1, int max_size = std::numeric_limits<int>::max()) {
  const int n = static_cast<int>(pts.size());
  if (n == 0) return {};
  auto key = [r](const Eigen::Vector3d& p) {
    // floor (not truncation) so negative coordinates land in the right cell.
    return Eigen::Vector3i(static_cast<int>(std::floor(p.x() / r)), static_cast<int>(std::floor(p.y() / r)),
                           static_cast<int>(std::floor(p.z() / r)));
  };
  // Voxel side = r: any pair within r lies in the same or an adjacent voxel.
  std::unordered_map<Eigen::Vector3i, std::vector<int>, VoxelHash, VoxelEq> grid;
  grid.reserve(n);
  for (int i = 0; i < n; ++i) grid[key(pts[i])].push_back(i);

  UnionFind uf(n);
  const double r2 = r * r;
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3i k = key(pts[i]);
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          auto it = grid.find(k + Eigen::Vector3i(dx, dy, dz));
          if (it == grid.end()) continue;
          for (int j : it->second)
            if (j > i && (pts[i] - pts[j]).squaredNorm() <= r2) uf.unite(i, j);
        }
  }

  // Canonical labels: clusters numbered by their lowest point index.
  std::vector<int> label(n, -1);
  std::unordered_map<int, int> root_to_label;
  int next = 0;
  for (int i = 0; i < n; ++i) {
    const int root = uf.find(i);
    const int sz = uf.size(root);
    if (sz < min_size || sz > max_size) continue;
    auto [it, inserted] = root_to_label.emplace(root, next);
    if (inserted) ++next;
    label[i] = it->second;
  }
  return label;
}
