#include <cmath>
#include <limits>
#include <vector>

struct Point3 { double x, y, z; };

double range(const Point3& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

void scaleInPlace(std::vector<Point3>& cloud, double s) {
  for (auto& p : cloud) { p.x *= s; p.y *= s; p.z *= s; }
}

const Point3* findClosest(const std::vector<Point3>& cloud, const Point3& q) {
  const Point3* best = nullptr;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (const auto& p : cloud) {
    const double dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
    const double d2 = dx * dx + dy * dy + dz * dz;  // no sqrt needed for comparison
    if (d2 < best_d2) { best_d2 = d2; best = &p; }
  }
  return best;
}

double pathLength(const std::vector<Point3>& cloud) {
  double total = 0.0;
  for (std::size_t i = 1; i < cloud.size(); ++i) {
    const double dx = cloud[i].x - cloud[i - 1].x, dy = cloud[i].y - cloud[i - 1].y,
                 dz = cloud[i].z - cloud[i - 1].z;
    total += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return total;
}
