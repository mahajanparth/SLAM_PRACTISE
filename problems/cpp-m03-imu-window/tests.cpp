// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  ImuWindow w(1.0);
  CHECK(w.push({0.0, 1, 0, 0}));
  CHECK(w.push({0.5, 3, 0, 2}));
  CHECK(w.push({1.0, 5, 0, 4}));
  CHECK_EQ(w.size(), std::size_t(3));  // 0.0 >= 1.0 - 1.0: kept (inclusive)
  CHECK(w.push({1.25, 7, 0, 6}));      // evicts t = 0.0
  CHECK_EQ(w.size(), std::size_t(3));
  auto [t0, t1] = w.timeSpan();
  CHECK_NEAR(t0, 0.5, 1e-12);
  CHECK_NEAR(t1, 1.25, 1e-12);
  auto m = w.meanGyro();
  CHECK_NEAR(m[0], 5.0, 1e-12);
  CHECK_NEAR(m[2], 4.0, 1e-12);
  CHECK(!w.push({1.0, 0, 0, 0}));  // out of order: rejected
  CHECK_EQ(w.size(), std::size_t(3));
  auto seg = w.between(0.6, 1.25);
  CHECK(seg.size() == 2);
  if (seg.size() == 2) CHECK_NEAR(seg[0].t, 1.0, 1e-12);

#ifdef HIDDEN
  // empty window
  ImuWindow e(0.5);
  CHECK(e.size() == 0);
  CHECK(e.timeSpan() == std::make_pair(0.0, 0.0));
  CHECK(e.meanGyro() == (std::array<double, 3>{0, 0, 0}));
  CHECK(e.between(-1, 1).empty());
  // equal timestamps accepted; reversed interval empty
  CHECK(e.push({2.0, 1, 1, 1}));
  CHECK(e.push({2.0, 3, 3, 3}));
  CHECK(e.size() == 2);
  CHECK(e.between(3.0, 1.0).empty());
  CHECK(e.between(2.0, 2.0).size() == 2);
  // zero-length window keeps only the newest timestamp
  ImuWindow z(0.0);
  z.push({1.0, 1, 0, 0});
  z.push({1.1, 2, 0, 0});
  CHECK(z.size() == 1);
  CHECK_NEAR(z.meanGyro()[0], 2.0, 1e-12);
  // a big jump in time evicts everything but the new sample
  z.push({100.0, -4, 0, 0});
  CHECK(z.size() == 1 && z.meanGyro()[0] == -4.0);
  // long run at 200 Hz: size stays bounded, the running mean stays exact
  ImuWindow big(1.0);
  for (int i = 0; i < 200000; ++i) big.push({i * 0.005, double(i % 7), -2.0, 0.25});
  CHECK(big.size() >= 200 && big.size() <= 202);
  double s = 0; std::size_t n = 0;
  for (const auto& smp : big.between(-1e9, 1e9)) { s += smp.gx; ++n; }
  CHECK(n == big.size());
  CHECK_NEAR(big.meanGyro()[0], s / n, 1e-9);
  CHECK_NEAR(big.meanGyro()[1], -2.0, 1e-9);
#endif
  return TEST_SUMMARY();
}
