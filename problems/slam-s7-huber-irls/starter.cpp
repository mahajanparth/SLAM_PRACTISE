#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

double huberWeight(double r, double k) {
  // TODO
  return 0.0;
}

double madSigma(std::vector<double> r) {
  // TODO: 1.4826 * median(|r - median(r)|)
  return 0.0;
}

Eigen::Vector2d fitLineLS(const std::vector<Eigen::Vector2d>& pts) {
  // TODO: returns (m, c)
  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d fitLineHuber(const std::vector<Eigen::Vector2d>& pts, double k, int iters = 20,
                             std::vector<double>* weights = nullptr) {
  // TODO: IRLS
  return Eigen::Vector2d::Zero();
}
