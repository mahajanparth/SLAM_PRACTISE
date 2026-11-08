#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

bool fitSphere(const Cloud& pts, Eigen::Vector3d& center, double& radius) {
  // TODO
  return false;
}

bool ransacSphere(const Cloud& pts, double thresh, int iters,
                  Eigen::Vector3d& center, double& radius, std::vector<int>& inliers,
                  unsigned seed = 42) {
  // TODO
  return false;
}
