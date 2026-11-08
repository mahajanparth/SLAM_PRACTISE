#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Vector3d triangulateDLT(const Eigen::Isometry3d& T1_cw, const Eigen::Vector2d& x1,
                               const Eigen::Isometry3d& T2_cw, const Eigen::Vector2d& x2) {
  // TODO
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d triangulateMidpoint(const Eigen::Isometry3d& T1_cw, const Eigen::Vector2d& x1,
                                    const Eigen::Isometry3d& T2_cw, const Eigen::Vector2d& x2) {
  // TODO
  return Eigen::Vector3d::Zero();
}

bool cheiralityOK(const Eigen::Isometry3d& T1_cw, const Eigen::Isometry3d& T2_cw,
                  const Eigen::Vector3d& X_w) {
  // TODO
  return false;
}

double parallaxAngle(const Eigen::Isometry3d& T1_cw, const Eigen::Isometry3d& T2_cw,
                     const Eigen::Vector3d& X_w) {
  // TODO
  return 0.0;
}
