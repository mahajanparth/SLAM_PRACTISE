#include <Eigen/Dense>
#include <cmath>
#include <vector>

struct GNResult {
  Eigen::Vector3d params = Eigen::Vector3d::Zero();  // (a, b, c)
  int iterations = 0;
  bool converged = false;
  double final_cost = 0.0;
};

GNResult fitExpQuadratic(const std::vector<double>& x, const std::vector<double>& y,
                         const Eigen::Vector3d& init = Eigen::Vector3d::Zero(), int max_iters = 50) {
  GNResult res;
  res.params = init;
  // TODO: Gauss-Newton loop
  return res;
}
