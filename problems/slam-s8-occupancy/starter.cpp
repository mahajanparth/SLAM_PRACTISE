#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <vector>

std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) {
  // TODO: inclusive, start -> end, all octants
  return {};
}

class OccupancyGrid {
 public:
  static constexpr double kLogFree = -0.4, kLogOcc = 0.85, kLogMin = -2.0, kLogMax = 3.5;

  OccupancyGrid(int width, int height, double resolution, const Eigen::Vector2d& origin)
      : width_(width), height_(height), res_(resolution), origin_(origin),
        data_(static_cast<size_t>(width) * height, 0.0) {}

  bool inBounds(int cx, int cy) const { return cx >= 0 && cy >= 0 && cx < width_ && cy < height_; }
  double logOdds(int cx, int cy) const { return inBounds(cx, cy) ? data_[idx(cx, cy)] : 0.0; }

  bool worldToCell(const Eigen::Vector2d& p, int& cx, int& cy) const {
    // TODO
    return false;
  }

  double probability(int cx, int cy) const {
    // TODO
    return 0.0;
  }

  void integrateScan(const Eigen::Vector3d& pose, const std::vector<double>& ranges,
                     double angle_min, double angle_increment, double max_range) {
    // TODO
  }

 private:
  size_t idx(int cx, int cy) const { return static_cast<size_t>(cy) * width_ + cx; }
  int width_, height_;
  double res_;
  Eigen::Vector2d origin_;
  std::vector<double> data_;
};
