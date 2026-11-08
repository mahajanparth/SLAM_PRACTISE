#include <Eigen/Dense>
#include <cmath>
#include <vector>

struct ImuState  { Eigen::Matrix3d R; Eigen::Vector3d v, p; };
struct ImuSample { double dt; Eigen::Vector3d gyro, accel; };

static Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
  Eigen::Matrix3d W;
  W << 0, -w.z(), w.y(),
       w.z(), 0, -w.x(),
      -w.y(), w.x(), 0;
  return W;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1e-8) return Eigen::Matrix3d::Identity() + hat(phi);  // first order; avoids 0/0
  const Eigen::Matrix3d K = hat(phi / theta);
  return Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * K * K;
}

ImuState integrateImu(ImuState s, const std::vector<ImuSample>& samples,
                      const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias,
                      const Eigen::Vector3d& gravity) {
  for (const auto& m : samples) {
    const double dt = m.dt;
    // World acceleration uses the orientation at the START of the interval.
    const Eigen::Vector3d a_w = s.R * (m.accel - accel_bias) + gravity;
    s.p += s.v * dt + 0.5 * a_w * dt * dt;   // uses old v
    s.v += a_w * dt;
    s.R = s.R * expSO3((m.gyro - gyro_bias) * dt);  // body-frame rate -> right multiply
  }
  // Optional hygiene for very long runs: re-orthonormalize (Rodrigues keeps it near-exact anyway).
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(s.R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  s.R = svd.matrixU() * svd.matrixV().transpose();
  return s;
}
