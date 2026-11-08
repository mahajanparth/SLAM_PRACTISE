#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

struct Point3 { double x, y, z; };

void removeOutOfRange(std::vector<Point3>& cloud, double min_r, double max_r) {
  const double lo2 = min_r * min_r, hi2 = max_r * max_r;  // compare squared: no sqrt per point
  auto outside = [&](const Point3& p) {
    const double r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    return r2 < lo2 || r2 > hi2;
  };
  // remove_if compacts survivors to the front and returns the new logical end.
  // erase then destroys the tail. One pass, stable.
  cloud.erase(std::remove_if(cloud.begin(), cloud.end(), outside), cloud.end());
}

void removeEveryNth(std::vector<Point3>& cloud, std::size_t n) {
  if (n == 0) return;  // % 0 would be UB
  std::size_t w = 0;   // write index never passes read index i, so this is safe in place
  for (std::size_t i = 0; i < cloud.size(); ++i)
    if ((i + 1) % n != 0) cloud[w++] = cloud[i];
  cloud.resize(w);
}

std::size_t dedupConsecutive(std::vector<Point3>& cloud, double eps) {
  // Same write-index pattern as removeEveryNth: compare each candidate with the LAST KEPT point.
  // (std::unique with this predicate works in practice, but "within eps" is not an equivalence
  //  relation, so the standard doesn't pin down which neighbour it compares against. Be explicit.)
  if (cloud.empty()) return 0;
  const double eps2 = eps * eps;
  std::size_t w = 1;  // cloud[0] is always kept
  for (std::size_t i = 1; i < cloud.size(); ++i) {
    const Point3& k = cloud[w - 1];
    const double dx = cloud[i].x - k.x, dy = cloud[i].y - k.y, dz = cloud[i].z - k.z;
    if (dx * dx + dy * dy + dz * dz > eps2) cloud[w++] = cloud[i];
  }
  const std::size_t removed = cloud.size() - w;
  cloud.resize(w);
  return removed;
}
