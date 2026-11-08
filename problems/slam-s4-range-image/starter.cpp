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
  img.range.assign(rows * cols, -1.0);
  img.index.assign(rows * cols, -1);
  // TODO
  return img;
}
