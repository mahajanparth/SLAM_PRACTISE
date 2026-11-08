#include <Eigen/Dense>
#include <cmath>
#include <vector>

struct Pose2 { double x = 0, y = 0, theta = 0; };
struct Edge2 {
  int i = 0, j = 0;
  Pose2 z;  // measured x_i^{-1} x_j
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
};

Eigen::Vector3d edgeError(const Pose2& xi, const Pose2& xj, const Pose2& z) {
  // TODO
  return Eigen::Vector3d::Zero();
}

double graphCost(const std::vector<Pose2>& poses, const std::vector<Edge2>& edges) {
  // TODO
  return 0.0;
}

int optimizePoseGraph(std::vector<Pose2>& poses, const std::vector<Edge2>& edges, int max_iters = 20) {
  // TODO: Gauss-Newton with pose 0 fixed
  return 0;
}
