// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <limits>
#include <vector>
#include "common/check.h"
#include "common/cloud_utils.h"

static int bruteNearest(const cu::Cloud& c, const Eigen::Vector3d& q) {
  int best = -1;
  double bd = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < c.size(); ++i) {
    const double d = (c[i] - q).squaredNorm();
    if (d < bd) { bd = d; best = static_cast<int>(i); }
  }
  return best;
}
static std::vector<int> bruteRadius(const cu::Cloud& c, const Eigen::Vector3d& q, double r) {
  std::vector<int> out;
  for (size_t i = 0; i < c.size(); ++i)
    if ((c[i] - q).norm() <= r) out.push_back(static_cast<int>(i));
  return out;
}
static std::vector<int> sorted(std::vector<int> v) { std::sort(v.begin(), v.end()); return v; }

// Fraction of random queries where nearest() returns a point at the brute-force distance.
static bool nearestMatches(const KDTree& t, const cu::Cloud& c, int queries, double half) {
  cu::Cloud qs = cu::uniformCube(queries, half);
  for (const auto& q : qs) {
    const int k = t.nearest(q), b = bruteNearest(c, q);
    if (k < 0 || k >= static_cast<int>(c.size())) return false;
    if (std::abs((c[k] - q).squaredNorm() - (c[b] - q).squaredNorm()) > 1e-12) return false;
  }
  return true;
}
static bool radiusMatches(const KDTree& t, const cu::Cloud& c, int queries, double half, double r) {
  cu::Cloud qs = cu::uniformCube(queries, half);
  for (const auto& q : qs)
    if (sorted(t.radiusSearch(q, r)) != bruteRadius(c, q, r)) return false;
  return true;
}

int main() {
  // Tiny hand-checkable cloud.
  cu::Cloud small = {{0, 0, 0}, {1, 0, 0}, {0, 2, 0}, {0, 0, 3}, {-1, -1, -1}};
  KDTree ts(small);
  CHECK_EQ(ts.nearest({0.9, 0.1, 0}), 1);
  CHECK_EQ(ts.nearest({0, 0, 10}), 3);
  CHECK(sorted(ts.radiusSearch({0, 0, 0}, 1.0)) == std::vector<int>({0, 1}));
  CHECK(ts.radiusSearch({50, 50, 50}, 1.0).empty());

  // Random cloud vs brute force.
  cu::Cloud c = cu::uniformCube(2000, 5.0);
  KDTree t(c);
  CHECK(nearestMatches(t, c, 200, 5.0));
  CHECK(radiusMatches(t, c, 100, 5.0, 0.8));

#ifdef HIDDEN
  // Empty tree.
  KDTree empty(cu::Cloud{});
  CHECK_EQ(empty.nearest({0, 0, 0}), -1);
  CHECK(empty.radiusSearch({0, 0, 0}, 10.0).empty());

  // Single point.
  KDTree one(cu::Cloud{{1, 2, 3}});
  CHECK_EQ(one.nearest({-100, 7, 1e6}), 0);
  CHECK(one.radiusSearch({1, 2, 3}, 0.0) == std::vector<int>({0}));

  // Many duplicates (degenerate splits must still terminate and be correct).
  cu::Cloud dup(500, Eigen::Vector3d(1, 1, 1));
  for (int i = 0; i < 100; ++i) dup.push_back(Eigen::Vector3d(2, 1, 1));
  KDTree td(dup);
  { const int k = td.nearest({1.1, 1, 1}); CHECK(k >= 0 && k < 500); }
  CHECK_EQ(td.radiusSearch({1, 1, 1}, 0.0).size(), size_t(500));
  CHECK_EQ(td.radiusSearch({1.5, 1, 1}, 0.5).size(), size_t(600));

  // Queries far outside the bounds, flat (planar) cloud, negative coords, larger radius.
  CHECK(nearestMatches(t, c, 80, 60.0));
  cu::Cloud flat = cu::planePoints(1500, {0, 0, 1}, 3.0, 4.0);
  KDTree tf(flat);
  CHECK(nearestMatches(tf, flat, 120, 5.0));
  CHECK(radiusMatches(tf, flat, 50, 5.0, 1.5));

  // r = 0 with query exactly on a point.
  CHECK(sorted(t.radiusSearch(c[1534], 0.0)) == bruteRadius(c, c[1534], 0.0));
  CHECK_EQ(t.nearest(c[777]), 777);
#endif
  return TEST_SUMMARY();
}
