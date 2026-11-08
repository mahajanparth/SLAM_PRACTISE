#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

// Shepperd's method: compute the largest of |w|,|x|,|y|,|z| from the diagonal first, so we
// always divide by a number >= 1/2 (never by ~0, as the naive trace formula does at 180°).
Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R) {
  const double tr = R.trace();
  double w, x, y, z;
  if (tr > 0.0) {                                   // |w| is the largest
    const double s = 2.0 * std::sqrt(1.0 + tr);     // s = 4w
    w = 0.25 * s;
    x = (R(2, 1) - R(1, 2)) / s;
    y = (R(0, 2) - R(2, 0)) / s;
    z = (R(1, 0) - R(0, 1)) / s;
  } else if (R(0, 0) >= R(1, 1) && R(0, 0) >= R(2, 2)) {   // |x| is the largest
    const double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));  // s = 4x
    w = (R(2, 1) - R(1, 2)) / s;
    x = 0.25 * s;
    y = (R(0, 1) + R(1, 0)) / s;
    z = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) >= R(2, 2)) {                  // |y| is the largest
    const double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));  // s = 4y
    w = (R(0, 2) - R(2, 0)) / s;
    x = (R(0, 1) + R(1, 0)) / s;
    y = 0.25 * s;
    z = (R(1, 2) + R(2, 1)) / s;
  } else {                                          // |z| is the largest
    const double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));  // s = 4z
    w = (R(1, 0) - R(0, 1)) / s;
    x = (R(0, 2) + R(2, 0)) / s;
    y = (R(1, 2) + R(2, 1)) / s;
    z = 0.25 * s;
  }
  Eigen::Vector4d q(w, x, y, z);
  q.normalize();            // absorb small non-orthonormality of R
  if (q(0) < 0) q = -q;     // canonical hemisphere (double cover)
  return q;
}

Eigen::Matrix3d quatToRot(const Eigen::Vector4d& qin) {
  const Eigen::Vector4d q = qin.normalized();
  const double w = q(0), x = q(1), y = q(2), z = q(3);
  Eigen::Matrix3d R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
       2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
       2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y);
  return R;
}
