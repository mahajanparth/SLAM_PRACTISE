#include <Eigen/Dense>
#include <cmath>

struct EkfState {
  Eigen::Vector3d x;   // (x, y, theta)
  Eigen::Matrix3d P;   // covariance
};

double wrapAngle(double a) {
  // atan2 returns (-pi, pi]; robust for any magnitude (no while-loops).
  return std::atan2(std::sin(a), std::cos(a));
}

void ekfPredict(EkfState& s, double v, double w, double dt, const Eigen::Matrix3d& Q) {
  const double th = s.x(2);
  const double c = std::cos(th), sn = std::sin(th);

  // Jacobian of the motion model wrt the state, evaluated at the OLD state.
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -v * dt * sn;
  F(1, 2) = v * dt * c;

  s.x(0) += v * dt * c;
  s.x(1) += v * dt * sn;
  s.x(2) = wrapAngle(th + w * dt);

  s.P = F * s.P * F.transpose() + Q;
  s.P = 0.5 * (s.P + s.P.transpose());
}

void ekfUpdate(EkfState& s, const Eigen::Vector2d& z, const Eigen::Vector2d& landmark,
               const Eigen::Matrix2d& R) {
  const Eigen::Vector2d d = landmark - s.x.head<2>();
  const double q = d.squaredNorm();
  const double r = std::sqrt(q);

  const Eigen::Vector2d h(r, wrapAngle(std::atan2(d.y(), d.x()) - s.x(2)));

  Eigen::Matrix<double, 2, 3> H;
  H << -d.x() / r, -d.y() / r, 0.0,
        d.y() / q, -d.x() / q, -1.0;

  Eigen::Vector2d y = z - h;
  y(1) = wrapAngle(y(1));                          // the classic bug if omitted

  const Eigen::Matrix2d S = H * s.P * H.transpose() + R;
  const Eigen::Matrix<double, 3, 2> K = s.P * H.transpose() * S.inverse();  // 2x2 inverse is fine

  s.x += K * y;
  s.x(2) = wrapAngle(s.x(2));

  // Joseph form: symmetric PSD for any K, robust to round-off.
  const Eigen::Matrix3d IKH = Eigen::Matrix3d::Identity() - K * H;
  s.P = IKH * s.P * IKH.transpose() + K * R * K.transpose();
  s.P = 0.5 * (s.P + s.P.transpose());
}
