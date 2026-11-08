#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

bool ransacPlane(const Cloud& pts, double thresh, int iters,
                 Eigen::Vector4d& plane, std::vector<int>& inliers, unsigned seed = 42) {
  // TODO
  return false;
}
