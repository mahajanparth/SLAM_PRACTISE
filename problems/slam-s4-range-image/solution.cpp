#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

struct RangeImage {
  int rows = 0, cols = 0;
  std::vector<double> range;   // rows*cols, row-major; -1 for empty pixels
  std::vector<int> index;      // rows*cols; index of the stored point, -1 if empty
  double at(int r, int c) const { return range[r * cols + c]; }
};

RangeImage projectToRangeImage(const std::vector<Eigen::Vector3d>& pts, int rows = 32, int cols = 1800,
                               double fov_down_deg = -30.0, double fov_up_deg = 10.0) {
  RangeImage img;
  img.rows = rows;
  img.cols = cols;
  img.range.assign(static_cast<std::size_t>(rows) * cols, -1.0);
  img.index.assign(static_cast<std::size_t>(rows) * cols, -1);

  const double fov_down = fov_down_deg * M_PI / 180.0, fov_up = fov_up_deg * M_PI / 180.0;
  const double fov = fov_up - fov_down;

  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    const Eigen::Vector3d& p = pts[i];
    const double r = p.norm();
    if (r < 1e-6) continue;                                        // azimuth/elevation undefined
    const double el = std::atan2(p.z(), std::hypot(p.x(), p.y()));
    if (el < fov_down || el > fov_up) continue;                    // outside the vertical FOV
    const double az = std::atan2(p.y(), p.x());                    // [-pi, pi]

    const int col = std::min(static_cast<int>(std::floor((az + M_PI) / (2.0 * M_PI) * cols)), cols - 1);
    const int row = std::min(static_cast<int>(std::floor((fov_up - el) / fov * rows)), rows - 1);
    const int px = row * cols + col;
    if (img.index[px] < 0 || r < img.range[px]) {                  // keep nearest (occlusion)
      img.range[px] = r;
      img.index[px] = i;
    }
  }
  return img;
}
