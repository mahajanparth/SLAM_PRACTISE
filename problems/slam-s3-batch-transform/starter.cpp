#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

Eigen::Matrix3Xd transformCloud(const Eigen::Isometry3d& T, const Eigen::Matrix3Xd& P) {
  // TODO
  return P;
}

std::vector<Eigen::Vector3d> transformCloud(const Eigen::Isometry3d& T, const std::vector<Eigen::Vector3d>& pts) {
  // TODO
  return pts;
}

Eigen::Matrix3Xf transformRaw(const Eigen::Isometry3d& T, const float* xyzi, int n) {
  // TODO
  return Eigen::Matrix3Xf::Zero(3, n);
}
