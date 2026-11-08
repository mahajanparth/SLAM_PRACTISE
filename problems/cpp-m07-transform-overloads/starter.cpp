#include <cmath>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };
struct Pose { double yaw, tx, ty, tz; };

Pose makePose(double yaw, double tx = 0, double ty = 0, double tz = 0) {
  // TODO
  return Pose{0, 0, 0, 0};
}

Point3 apply(const Pose& T, const Point3& p) {
  // TODO: R_z(yaw) * p + t
  return Point3{0, 0, 0};
}

void applyInPlace(const Pose& T, std::vector<Point3>& cloud) {
  // TODO
}

std::vector<Point3> apply(const Pose& T, const std::vector<Point3>& cloud) {
  // TODO: transformed copy
  return {};
}

std::vector<Point3> apply(const Pose& T, std::vector<Point3>&& cloud) {
  // TODO: reuse the caller's buffer
  return {};
}
