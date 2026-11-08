#include <Eigen/Dense>
#include <Eigen/Geometry>

struct SE3 {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  SE3 operator*(const SE3& other) const {
    // TODO
    return SE3{};
  }
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const {
    // TODO
    return p;
  }
  SE3 inverse() const {
    // TODO
    return SE3{};
  }
  Eigen::Matrix4d matrix() const {
    // TODO
    return Eigen::Matrix4d::Identity();
  }
};
