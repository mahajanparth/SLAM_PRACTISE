// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

static std::vector<Point3> makeCloud(std::size_t n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(-20, 20);
  std::vector<Point3> c(n);
  for (auto& p : c) p = {U(rng), U(rng), U(rng) * 0.1 + 1.0};
  return c;
}

int main() {
  std::vector<Point3> sq = {{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 4}};
  Point3 c = parallelCentroid(sq, 2);
  CHECK_NEAR(c.x, 1.0, 1e-12);
  CHECK_NEAR(c.y, 1.0, 1e-12);
  CHECK_NEAR(c.z, 1.0, 1e-12);

  CHECK_EQ(countInRangeParallel(sq, 2.0, 3), 2u);  // (0,0,0) and (2,0,0)

  auto cloud = makeCloud(20000, 1);
  double sx = 0;
  for (const auto& p : cloud) sx += p.x;
  CHECK_NEAR(parallelCentroid(cloud, 4).x, sx / cloud.size(), 1e-9);

  HitCounter hc;
  std::vector<std::thread> ts;
  for (int t = 0; t < 4; ++t)
    ts.emplace_back([&hc] { for (int i = 0; i < 1000; ++i) hc.add(i % 10); });
  for (auto& th : ts) th.join();
  CHECK_EQ(hc.get(3), 400);
  CHECK_EQ(hc.distinct(), 10u);

#ifdef HIDDEN
  // empty, single point, more threads than points, non-positive thread counts
  Point3 e = parallelCentroid({}, 4);
  CHECK(e.x == 0 && e.y == 0 && e.z == 0);
  CHECK_EQ(countInRangeParallel({}, 5.0, 4), 0u);
  Point3 one = parallelCentroid({{-1, 2, -3}}, 8);
  CHECK(one.x == -1 && one.y == 2 && one.z == -3);
  Point3 z = parallelCentroid(sq, 0);
  CHECK_NEAR(z.y, 1.0, 1e-12);
  CHECK_EQ(countInRangeParallel(sq, 100.0, -3), 4u);
  CHECK_EQ(countInRangeParallel(sq, 2.0, 16), 2u);
  // uneven chunks: n not divisible by T, compare with serial
  auto odd = makeCloud(10007, 9);
  double ox = 0, oy = 0, oz = 0;
  std::size_t serial = 0;
  for (const auto& p : odd) {
    ox += p.x; oy += p.y; oz += p.z;
    if (std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) <= 15.0) ++serial;
  }
  bool ok = true;
  for (int T : {1, 3, 7, 8}) {
    Point3 m = parallelCentroid(odd, T);
    ok = ok && std::abs(m.x - ox / odd.size()) < 1e-9 && std::abs(m.y - oy / odd.size()) < 1e-9 &&
         std::abs(m.z - oz / odd.size()) < 1e-9;
    ok = ok && countInRangeParallel(odd, 15.0, T) == serial;
  }
  CHECK(ok);
  // HitCounter: concurrent add with n, readers in parallel, unknown cell
  HitCounter h2;
  std::vector<std::thread> w;
  for (int t = 0; t < 4; ++t)
    w.emplace_back([&h2, t] { for (int i = 0; i < 2000; ++i) { h2.add(i % 50 - 25, t + 1); (void)h2.get(0); } });
  for (auto& th : w) th.join();
  CHECK_EQ(h2.get(-25), 40 * (1 + 2 + 3 + 4));
  CHECK_EQ(h2.distinct(), 50u);
  CHECK_EQ(h2.get(999), 0);
  const HitCounter& ch = h2;
  CHECK_EQ(ch.get(24), 400);
#endif
  return TEST_SUMMARY();
}
