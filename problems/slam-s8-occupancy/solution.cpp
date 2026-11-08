#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) {
  // Integer error-accumulating form that handles all octants without swapping.
  std::vector<std::pair<int, int>> cells;
  const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  cells.reserve(std::max(dx, -dy) + 1);
  int err = dx + dy;
  while (true) {
    cells.emplace_back(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
  return cells;
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
    // floor, not a cast: (int)(-0.3) == 0 would merge cells -1 and 0.
    cx = static_cast<int>(std::floor((p.x() - origin_.x()) / res_));
    cy = static_cast<int>(std::floor((p.y() - origin_.y()) / res_));
    return inBounds(cx, cy);
  }

  double probability(int cx, int cy) const {
    return 1.0 - 1.0 / (1.0 + std::exp(logOdds(cx, cy)));
  }

  void integrateScan(const Eigen::Vector3d& pose, const std::vector<double>& ranges,
                     double angle_min, double angle_increment, double max_range) {
    int rx, ry;
    worldToCell(pose.head<2>(), rx, ry);  // robot may be outside the map; still trace
    for (size_t i = 0; i < ranges.size(); ++i) {
      const double r = ranges[i];
      if (!std::isfinite(r) || r <= 0.0) continue;
      const bool hit = r < max_range;
      const double len = hit ? r : max_range;
      const double a = pose.z() + angle_min + static_cast<double>(i) * angle_increment;
      const Eigen::Vector2d end = pose.head<2>() + len * Eigen::Vector2d(std::cos(a), std::sin(a));
      int ex, ey;
      worldToCell(end, ex, ey);
      const auto ray = bresenham(rx, ry, ex, ey);
      for (size_t k = 0; k < ray.size(); ++k) {
        const auto [cx, cy] = ray[k];
        if (!inBounds(cx, cy)) continue;
        const bool last = (k + 1 == ray.size());
        update(cx, cy, (hit && last) ? kLogOcc : kLogFree);
      }
    }
  }

 private:
  size_t idx(int cx, int cy) const { return static_cast<size_t>(cy) * width_ + cx; }
  void update(int cx, int cy, double l) {
    double& v = data_[idx(cx, cy)];
    v = std::clamp(v + l, kLogMin, kLogMax);  // clamping keeps the map able to change
  }
  int width_, height_;
  double res_;
  Eigen::Vector2d origin_;
  std::vector<double> data_;
};
