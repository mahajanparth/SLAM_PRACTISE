#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace {
// Shared core. Returns false on invalid input. Outputs R (proper), s, t for q ≈ s R p + t.
bool alignCore(const Eigen::Matrix3Xd& P, const Eigen::Matrix3Xd& Q, bool with_scale,
               Eigen::Matrix3d& R, double& s, Eigen::Vector3d& t) {
  const Eigen::Index n = P.cols();
  if (n != Q.cols() || n < 3) return false;
  const Eigen::Vector3d mp = P.rowwise().mean(), mq = Q.rowwise().mean();
  const Eigen::Matrix3Xd Pc = P.colwise() - mp, Qc = Q.colwise() - mq;

  const Eigen::Matrix3d H = Pc * Qc.transpose();  // Σ p̃ q̃ᵀ
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d& U = svd.matrixU();
  const Eigen::Matrix3d& V = svd.matrixV();

  // Reflection guard: flip the axis of the smallest singular value if det(V Uᵀ) = -1.
  Eigen::Vector3d D(1.0, 1.0, (V * U.transpose()).determinant() < 0 ? -1.0 : 1.0);
  R = V * D.asDiagonal() * U.transpose();

  s = 1.0;
  if (with_scale) {
    const double var_p = Pc.squaredNorm();  // Σ |p̃|²
    if (var_p <= 0.0) return false;
    s = svd.singularValues().dot(D) / var_p;  // trace(Σ D) / Σ|p̃|²
  }
  t = mq - s * R * mp;  // translation uses the ROTATED source centroid
  return true;
}
}  // namespace

Eigen::Isometry3d kabsch(const Eigen::Matrix3Xd& P, const Eigen::Matrix3Xd& Q) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double s;
  if (!alignCore(P, Q, false, R, s, t)) return T;
  T.linear() = R;
  T.translation() = t;
  return T;
}

Eigen::Matrix4d umeyamaSim3(const Eigen::Matrix3Xd& P, const Eigen::Matrix3Xd& Q) {
  Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double s;
  if (!alignCore(P, Q, true, R, s, t)) return S;
  S.topLeftCorner<3, 3>() = s * R;
  S.topRightCorner<3, 1>() = t;
  return S;
}
