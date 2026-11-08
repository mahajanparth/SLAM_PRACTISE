#include <Eigen/Dense>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

bool fitPlane(const Cloud& pts, Eigen::Vector3d& normal, double& d) {
  // TODO
  return false;
}

bool fitLine(const Cloud& pts, Eigen::Vector3d& point, Eigen::Vector3d& dir) {
  // TODO
  return false;
}
