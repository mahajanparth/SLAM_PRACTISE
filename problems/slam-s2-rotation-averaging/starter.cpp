#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& qs,
                                      const std::vector<double>& weights) {
  // TODO
  return Eigen::Quaterniond::Identity();
}
