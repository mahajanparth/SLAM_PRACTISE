#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

namespace {
// Kabsch: argmin_{R,t} Σ |R p + t - q|², proper rotation guaranteed.
Eigen::Isometry3d kabsch(const Cloud& P, const Cloud& Q) {
  const double n = static_cast<double>(P.size());
  Eigen::Vector3d mp = Eigen::Vector3d::Zero(), mq = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < P.size(); ++i) { mp += P[i]; mq += Q[i]; }
  mp /= n;
  mq /= n;
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < P.size(); ++i) H += (P[i] - mp) * (Q[i] - mq).transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
  const Eigen::Vector3d D(1, 1, (V * U.transpose()).determinant() < 0 ? -1 : 1);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = V * D.asDiagonal() * U.transpose();
  T.translation() = mq - T.linear() * mp;
  return T;
}

// Brute-force NN on raw doubles (fast enough for a few hundred points even in debug builds).
int nearest(const Cloud& tgt, const Eigen::Vector3d& q, double& best_d2) {
  int best = -1;
  best_d2 = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < tgt.size(); ++j) {
    const double dx = tgt[j].x() - q.x(), dy = tgt[j].y() - q.y(), dz = tgt[j].z() - q.z();
    const double d2 = dx * dx + dy * dy + dz * dz;
    if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(j); }
  }
  return best;
}
}  // namespace

Eigen::Isometry3d icp(const Cloud& src, const Cloud& tgt, const Eigen::Isometry3d& init,
                      int max_iters,
                      double max_corr_dist = std::numeric_limits<double>::infinity()) {
  Eigen::Isometry3d T = init;
  if (src.empty() || tgt.empty()) return T;
  const double gate2 = max_corr_dist * max_corr_dist;  // inf stays inf
  Cloud P, Q;
  P.reserve(src.size());
  Q.reserve(src.size());

  for (int it = 0; it < max_iters; ++it) {
    // 1) Correspondences under the CURRENT estimate, with distance gating.
    P.clear();
    Q.clear();
    for (const auto& s : src) {
      const Eigen::Vector3d p = T * s;
      double d2;
      const int j = nearest(tgt, p, d2);
      if (d2 <= gate2) { P.push_back(p); Q.push_back(tgt[j]); }
    }
    if (P.size() < 3) break;

    // 2) Closed-form increment, applied on the left (P is already in the target frame).
    const Eigen::Isometry3d dT = kabsch(P, Q);
    T = dT * T;

    // 3) Converged when the increment is negligible.
    const double dang = Eigen::AngleAxisd(dT.linear()).angle();
    if (dang < 1e-10 && dT.translation().norm() < 1e-10) break;
  }
  // Re-orthonormalise to kill accumulated round-off in the rotation.
  T.linear() = Eigen::Quaterniond(T.linear()).normalized().toRotationMatrix();
  return T;
}
