#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Matrix3d eulerZYXToRot(double yaw, double pitch, double roll) {
  // TODO
  return Eigen::Matrix3d::Identity();
}

Eigen::Vector3d rotToEulerZYX(const Eigen::Matrix3d& R) {
  // TODO: return (yaw, pitch, roll)
  return Eigen::Vector3d::Zero();
}
