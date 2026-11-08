#include <Eigen/Dense>
#include <vector>

struct Intrinsics { double fx, fy, cx, cy, k1, k2, p1, p2; };

bool projectPoint(const Intrinsics& K, const Eigen::Vector3d& Pc, Eigen::Vector2d& uv,
                  double min_depth = 1e-6) {
  if (!(Pc.z() > min_depth)) return false;          // behind camera (also rejects NaN)
  const double x = Pc.x() / Pc.z(), y = Pc.y() / Pc.z();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + r2 * (K.k1 + K.k2 * r2);  // Horner: 1 + k1 r^2 + k2 r^4
  const double xd = x * radial + 2.0 * K.p1 * x * y + K.p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + K.p1 * (r2 + 2.0 * y * y) + 2.0 * K.p2 * x * y;
  uv << K.fx * xd + K.cx, K.fy * yd + K.cy;
  return true;
}

std::vector<int> projectToImage(const Intrinsics& K, const std::vector<Eigen::Vector3d>& cloud,
                                int width, int height, std::vector<Eigen::Vector2d>& pixels) {
  std::vector<int> idx;
  pixels.clear();
  Eigen::Vector2d uv;
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    if (!projectPoint(K, cloud[i], uv)) continue;
    if (uv.x() < 0 || uv.y() < 0 || uv.x() >= width || uv.y() >= height) continue;
    idx.push_back(i);
    pixels.push_back(uv);
  }
  return idx;
}
