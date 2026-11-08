#include <Eigen/Dense>
#include <vector>

struct Intrinsics { double fx, fy, cx, cy, k1, k2, p1, p2; };

bool projectPoint(const Intrinsics& K, const Eigen::Vector3d& Pc, Eigen::Vector2d& uv,
                  double min_depth = 1e-6) {
  // TODO
  return false;
}

std::vector<int> projectToImage(const Intrinsics& K, const std::vector<Eigen::Vector3d>& cloud,
                                int width, int height, std::vector<Eigen::Vector2d>& pixels) {
  // TODO
  return {};
}
