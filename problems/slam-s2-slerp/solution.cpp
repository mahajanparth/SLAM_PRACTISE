#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

Eigen::Quaterniond slerp(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double t) {
  Eigen::Vector4d qa = a.coeffs(), qb = b.coeffs();   // (x, y, z, w) -- order doesn't matter here
  double d = qa.dot(qb);
  if (d < 0.0) { qb = -qb; d = -d; }                 // double cover: take the shortest arc
  d = std::min(d, 1.0);

  Eigen::Vector4d r;
  if (d > 1.0 - 1e-10) {
    r = (1.0 - t) * qa + t * qb;                      // nearly equal: nlerp avoids 0/0
  } else {
    const double omega = std::acos(d);                // half of the rotation angle between a and b
    const double s = std::sin(omega);
    r = (std::sin((1.0 - t) * omega) / s) * qa + (std::sin(t * omega) / s) * qb;
  }
  r.normalize();
  Eigen::Quaterniond q;
  q.coeffs() = r;
  return q;
}
