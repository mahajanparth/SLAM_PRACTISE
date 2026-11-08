#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

// Given: brute-force kNN, sorted by distance, ties -> smaller index.
std::vector<int> knn(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector3d& q, int k) {
  if (k <= 0 || pts.empty()) return {};
  const int n = static_cast<int>(pts.size()), kk = std::min(k, n);
  std::vector<std::pair<double, int>> d(n);
  for (int i = 0; i < n; ++i) d[i] = {(pts[i] - q).squaredNorm(), i};
  std::partial_sort(d.begin(), d.begin() + kk, d.end());
  std::vector<int> out(kk);
  for (int i = 0; i < kk; ++i) out[i] = d[i].second;
  return out;
}

std::vector<int> statisticalOutlierRemoval(const std::vector<Eigen::Vector3d>& pts, int k, double alpha) {
  // TODO
  std::vector<int> all(pts.size());
  std::iota(all.begin(), all.end(), 0);
  return all;
}
