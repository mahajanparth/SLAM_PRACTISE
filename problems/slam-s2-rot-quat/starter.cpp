#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R) {
  // TODO: return (w, x, y, z), unit norm, w >= 0
  return Eigen::Vector4d(1, 0, 0, 0);
}

Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q) {
  // TODO: q = (w, x, y, z), possibly non-unit
  return Eigen::Matrix3d::Identity();
}
