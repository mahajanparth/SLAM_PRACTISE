#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

Vector6d pointToPlaneStep(const Cloud& p, const Cloud& q, const Cloud& n) {
  // TODO
  return Vector6d::Zero();
}

Eigen::Isometry3d applyUpdate(const Vector6d& delta, const Eigen::Isometry3d& T) {
  // TODO
  return T;
}
