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
  // TODO
  return Eigen::Matrix3d::Identity();
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  // TODO
  return Eigen::Vector3d::Zero();
}
