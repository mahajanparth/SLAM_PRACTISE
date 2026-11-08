#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <utility>
#include <vector>

struct Normal {
  Eigen::Vector3d n = Eigen::Vector3d::Zero();
  double curvature = 0.0;
  bool valid = false;
};

// Given: brute-force kNN (S4 problem 3), sorted by distance, ties -> smaller index.
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

std::vector<Normal> estimateNormals(const std::vector<Eigen::Vector3d>& pts, int k,
                                    const Eigen::Vector3d& viewpoint) {
  // TODO
  return std::vector<Normal>(pts.size());
}
