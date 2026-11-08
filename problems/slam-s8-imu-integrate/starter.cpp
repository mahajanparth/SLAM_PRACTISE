#include <Eigen/Dense>
#include <cmath>
#include <vector>

struct ImuState  { Eigen::Matrix3d R; Eigen::Vector3d v, p; };
struct ImuSample { double dt; Eigen::Vector3d gyro, accel; };

Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi) {
  // TODO: Rodrigues formula, small-angle safe
  return Eigen::Matrix3d::Identity();
}

ImuState integrateImu(ImuState s, const std::vector<ImuSample>& samples,
                      const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias,
                      const Eigen::Vector3d& gravity) {
  // TODO
  return s;
}
