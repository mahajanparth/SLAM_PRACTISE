#include <Eigen/Dense>
#include <Eigen/Geometry>

// [v]x such that [v]x * b == v x b.
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  return S;
}

// Inverse of skew. Averaging the mirrored entries returns vee of the skew part 0.5*(S - S^T),
// which is the right thing to do for a slightly-noisy input (e.g. R - R^T in logSO3).
Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
  return 0.5 * Eigen::Vector3d(S(2, 1) - S(1, 2), S(0, 2) - S(2, 0), S(1, 0) - S(0, 1));
}
