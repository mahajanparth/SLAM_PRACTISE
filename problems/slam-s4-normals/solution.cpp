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
  std::vector<Normal> out(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const std::vector<int> nb = knn(pts, pts[i], k);
    if (nb.size() < 3) continue;

    // Two-pass covariance: subtract the centroid first (no catastrophic cancellation far from the origin).
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (int j : nb) c += pts[j];
    c /= static_cast<double>(nb.size());
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (int j : nb) {
      const Eigen::Vector3d d = pts[j] - c;
      C.noalias() += d * d.transpose();
    }
    C /= static_cast<double>(nb.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);  // eigenvalues ascending
    const Eigen::Vector3d lam = es.eigenvalues().cwiseMax(0.0);  // clamp tiny negative round-off
    if (lam(2) <= 0.0 || lam(1) <= 1e-10 * lam(2)) continue;   // identical / collinear points

    Eigen::Vector3d n = es.eigenvectors().col(0).normalized();
    if (n.dot(viewpoint - pts[i]) < 0.0) n = -n;               // orient toward the sensor
    out[i].n = n;
    out[i].curvature = lam(0) / lam.sum();
    out[i].valid = true;
  }
  return out;
}
