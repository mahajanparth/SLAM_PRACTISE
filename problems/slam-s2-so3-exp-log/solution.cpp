#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

// Given.
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  const double th2 = w.squaredNorm();
  const Eigen::Matrix3d K = skew(w);
  if (th2 < 1e-16) {
    // Taylor: sin(t)/t ~ 1 - t^2/6, (1-cos t)/t^2 ~ 1/2 - t^2/24. Here t^2 < 1e-16, so higher terms vanish.
    return Eigen::Matrix3d::Identity() + K + 0.5 * K * K;
  }
  const double th = std::sqrt(th2);
  return Eigen::Matrix3d::Identity() + (std::sin(th) / th) * K + ((1.0 - std::cos(th)) / th2) * K * K;
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  // vee(R - R^T) = 2 sin(theta) * axis
  const Eigen::Vector3d v(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  const double s = 0.5 * v.norm();                                    // sin(theta) >= 0
  const double c = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);    // cos(theta), clamped
  const double th = std::atan2(s, c);                                 // accurate everywhere in [0, pi]

  if (th < 1e-8) return 0.5 * v;  // theta/(2 sin theta) -> 1/2

  if (M_PI - th < 1e-5) {
    // Near pi, sin(theta) ~ 0 so v carries no reliable axis information.
    // Symmetric part: S = (R + R^T)/2 = c I + (1 - c) a a^T  ->  a a^T = (S - c I) / (1 - c).
    const Eigen::Matrix3d S = 0.5 * (R + R.transpose());
    const Eigen::Matrix3d aaT = (S - c * Eigen::Matrix3d::Identity()) / (1.0 - c);
    int k;
    aaT.diagonal().maxCoeff(&k);                       // most reliable column (|a_k| largest)
    Eigen::Vector3d a = aaT.col(k) / std::sqrt(aaT(k, k));
    a.normalize();
    if (a.dot(v) < 0) a = -a;                           // sign from the (small) antisymmetric part
    return th * a;
  }
  return (th / (2.0 * std::sin(th))) * v;
}
