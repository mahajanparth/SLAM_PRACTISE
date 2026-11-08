#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

std::vector<int> removeGroundHeight(const std::vector<Eigen::Vector3d>& pts, double sensor_height, double threshold) {
  const double ground_limit = -sensor_height + threshold;
  std::vector<int> keep;
  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
    if (!(pts[i].z() < ground_limit)) keep.push_back(i);
  return keep;
}

struct CellHash {
  std::size_t operator()(const std::pair<int64_t, int64_t>& k) const {
    return static_cast<std::size_t>(k.first * 73856093LL) ^ static_cast<std::size_t>(k.second * 19349663LL);
  }
};

std::vector<int> removeGroundGrid(const std::vector<Eigen::Vector3d>& pts, double cell, double threshold) {
  if (!(cell > 0.0)) throw std::invalid_argument("cell must be > 0");
  auto key = [cell](const Eigen::Vector3d& p) {
    return std::make_pair(static_cast<int64_t>(std::floor(p.x() / cell)), static_cast<int64_t>(std::floor(p.y() / cell)));
  };
  // Pass 1: lowest point per XY cell (assumed to be ground if the cell sees any ground).
  std::unordered_map<std::pair<int64_t, int64_t>, double, CellHash> min_z;
  min_z.reserve(pts.size());
  for (const auto& p : pts) {
    auto it = min_z.find(key(p));
    if (it == min_z.end()) min_z.emplace(key(p), p.z());
    else it->second = std::min(it->second, p.z());
  }
  // Pass 2: points close to their cell's minimum are ground.
  std::vector<int> keep;
  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
    if (!(pts[i].z() - min_z.at(key(pts[i])) < threshold)) keep.push_back(i);
  return keep;
}
