// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  std::vector<Point3> c = {{1, 0, 0}, {3, 0, 0}, {0, 2, 0}, {0, 0, -4}};
  Point3 m = centroid(c);
  CHECK_NEAR(m.x, 1.0, 1e-12);
  CHECK_NEAR(m.z, -1.0, 1e-12);
  CHECK_EQ(countWithin(c, {0, 0, 0}, 2.0), std::size_t(2));  // inclusive

  std::vector<Point3> s = {{0, 3, 0}, {1, 0, 0}, {3, 0, 0}, {0, 0, 2}};  // ranges 3,1,3,2
  sortByRange(s);
  CHECK(s.size() == 4);
  if (s.size() == 4) {
    CHECK_NEAR(s[0].x, 1.0, 1e-12);
    CHECK_NEAR(s[2].y, 3.0, 1e-12);  // stable: (0,3,0) came before (3,0,0)
    CHECK_NEAR(s[3].x, 3.0, 1e-12);
  }

  auto k = kClosest(c, {0.9, 0, 0}, 2);
  CHECK(k == (std::vector<std::size_t>{0, 1}));

#ifdef HIDDEN
  // empty and k edge cases
  Point3 e = centroid({});
  CHECK(e.x == 0 && e.y == 0 && e.z == 0);
  CHECK(kClosest({}, {0, 0, 0}, 3).empty());
  CHECK(kClosest(c, {0, 0, 0}, 0).empty());
  CHECK(kClosest(c, {0, 0, 0}, 10) == (std::vector<std::size_t>{0, 2, 1, 3}));  // k > n
  CHECK_EQ(countWithin({}, {0, 0, 0}, 1.0), std::size_t(0));
  // ties -> smaller index
  std::vector<Point3> ring = {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}, {5, 5, 5}};
  CHECK(kClosest(ring, {0, 0, 0}, 3) == (std::vector<std::size_t>{0, 1, 2}));
  // large cloud vs brute force
  std::vector<Point3> big;
  for (int i = 0; i < 200000; ++i)
    big.push_back({double((i * 37) % 1001) - 500, double((i * 91) % 997) - 498, double(i % 13)});
  Point3 q{3.3, -7.1, 6.0};
  auto bk = kClosest(big, q, 25);
  std::vector<std::size_t> all(big.size());
  for (std::size_t i = 0; i < all.size(); ++i) all[i] = i;
  auto d2 = [&](std::size_t i) { double dx = big[i].x - q.x, dy = big[i].y - q.y, dz = big[i].z - q.z; return dx*dx+dy*dy+dz*dz; };
  std::stable_sort(all.begin(), all.end(), [&](std::size_t a, std::size_t b) { return d2(a) < d2(b); });
  all.resize(25);
  CHECK(bk == all);
  Point3 bm = centroid(big);
  CHECK_NEAR(bm.z, 6.0, 0.01);
  // stable sort on many equal ranges: unit vectors along different axes all have range exactly 1
  const Point3 axes[4] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}};
  std::vector<Point3> eq;
  for (int i = 0; i < 1000; ++i) eq.push_back(i % 2 ? Point3{0, 0, double(i + 1)} : axes[(i / 2) % 4]);
  sortByRange(eq);
  bool stable = true;
  for (int j = 0; j < 500; ++j) {
    const Point3& a = axes[j % 4];
    stable &= (eq[j].x == a.x && eq[j].y == a.y && eq[j].z == a.z);
  }
  for (int j = 501; j < 1000; ++j) stable &= (eq[j].z > eq[j - 1].z);
  CHECK(stable);
#endif
  return TEST_SUMMARY();
}
