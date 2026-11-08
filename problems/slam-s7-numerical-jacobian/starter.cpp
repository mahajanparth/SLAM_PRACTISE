#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>

// Provided: Rodrigues formula, Exp: so(3) -> SO(3).
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  const double th = w.norm();
  if (th < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, w / th).toRotationMatrix();
}

Eigen::MatrixXd numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
                                  const Eigen::VectorXd& x, double eps = 1e-6) {
  // TODO: central differences
  return Eigen::MatrixXd();
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  // TODO
  return Eigen::Matrix3d::Zero();
}

Eigen::Matrix3d rotatePointJacobianAnalytic(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
  // TODO: d(R Exp(delta) p)/d delta at delta = 0
  return Eigen::Matrix3d::Zero();
}

Eigen::Matrix3d rotatePointJacobianNumerical(const Eigen::Matrix3d& R, const Eigen::Vector3d& p,
                                             double eps = 1e-6) {
  // TODO: use numericalJacobian with a lambda over delta
  return Eigen::Matrix3d::Zero();
}
