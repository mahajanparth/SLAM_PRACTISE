#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

struct Point3 { double x, y, z; };

static double dist2(const Point3& a, const Point3& b) {
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

Point3 centroid(const std::vector<Point3>& c) {
  if (c.empty()) return {0, 0, 0};
  // The init value fixes the accumulator TYPE: Point3{0,0,0}, not 0.
  Point3 s = std::accumulate(c.begin(), c.end(), Point3{0, 0, 0}, [](Point3 acc, const Point3& p) {
    acc.x += p.x; acc.y += p.y; acc.z += p.z;
    return acc;
  });
  const double n = static_cast<double>(c.size());
  return {s.x / n, s.y / n, s.z / n};
}

std::size_t countWithin(const std::vector<Point3>& c, const Point3& q, double r) {
  const double r2 = r * r;
  return static_cast<std::size_t>(
      std::count_if(c.begin(), c.end(), [&](const Point3& p) { return dist2(p, q) <= r2; }));
}

void sortByRange(std::vector<Point3>& c) {
  const Point3 o{0, 0, 0};
  // stable_sort: O(n log n) (with a buffer), and equal keys keep their input order. std::sort gives no such promise.
  std::stable_sort(c.begin(), c.end(), [&](const Point3& a, const Point3& b) { return dist2(a, o) < dist2(b, o); });
}

std::vector<std::size_t> kClosest(const std::vector<Point3>& c, const Point3& q, std::size_t k) {
  const std::size_t n = c.size();
  k = std::min(k, n);
  std::vector<double> d2(n);
  std::transform(c.begin(), c.end(), d2.begin(), [&](const Point3& p) { return dist2(p, q); });
  std::vector<std::size_t> idx(n);
  std::iota(idx.begin(), idx.end(), std::size_t{0});
  // (distance, index) lexicographic: a strict total order, so the tie-break is deterministic.
  auto cmp = [&](std::size_t a, std::size_t b) { return d2[a] < d2[b] || (d2[a] == d2[b] && a < b); };
  if (k < n) std::nth_element(idx.begin(), idx.begin() + k, idx.end(), cmp);  // O(n) average
  std::sort(idx.begin(), idx.begin() + k, cmp);                               // O(k log k)
  idx.resize(k);
  return idx;
}
