#include <cmath>
#include <utility>
#include <vector>

struct Point3 { double x, y, z; };
struct Pose { double yaw, tx, ty, tz; };

// Default arguments: trailing parameters only. In a header they go on the declaration only.
Pose makePose(double yaw, double tx = 0, double ty = 0, double tz = 0) { return Pose{yaw, tx, ty, tz}; }

// Pose is 32 B, so pass by const&. Point3 is 24 B; by value would also be fine.
Point3 apply(const Pose& T, const Point3& p) {
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  return Point3{c * p.x - s * p.y + T.tx, s * p.x + c * p.y + T.ty, p.z + T.tz};
}

void applyInPlace(const Pose& T, std::vector<Point3>& cloud) {
  for (auto& p : cloud) p = apply(T, p);  // apply() returns a new value, so aliasing is not a problem
}

// Lvalue overload: the caller keeps its cloud, so we must produce a new one.
std::vector<Point3> apply(const Pose& T, const std::vector<Point3>& cloud) {
  std::vector<Point3> out;
  out.reserve(cloud.size());  // single allocation
  for (const auto& p : cloud) out.push_back(apply(T, p));
  return out;                 // NRVO, or an implicit move
}

// Rvalue overload: the caller handed us ownership, so reuse its buffer.
std::vector<Point3> apply(const Pose& T, std::vector<Point3>&& cloud) {
  applyInPlace(T, cloud);     // `cloud` has a name, so it's an lvalue here
  return std::move(cloud);    // required: implicit move does NOT apply to reference parameters (C++17)
}
