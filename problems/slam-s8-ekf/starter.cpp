#include <Eigen/Dense>
#include <cmath>

struct EkfState {
  Eigen::Vector3d x;   // (x, y, theta)
  Eigen::Matrix3d P;   // covariance
};

double wrapAngle(double a) {
  // TODO: map to (-pi, pi]
  return a;
}

void ekfPredict(EkfState& s, double v, double w, double dt, const Eigen::Matrix3d& Q) {
  // TODO: propagate mean with the unicycle model, covariance with F P F^T + Q
}

void ekfUpdate(EkfState& s, const Eigen::Vector2d& z, const Eigen::Vector2d& landmark,
               const Eigen::Matrix2d& R) {
  // TODO: range-bearing update (wrap the bearing innovation!)
}
