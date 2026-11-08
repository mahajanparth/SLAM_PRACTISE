// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <algorithm>
#include <random>

static double bruteMedian(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

int main() {
  RunningMedian m;
  m.add(5.0);
  CHECK_NEAR(m.median(), 5.0, 1e-12);
  m.add(1.0);
  CHECK_NEAR(m.median(), 3.0, 1e-12);
  m.add(100.0);  // spike barely moves the median
  CHECK_NEAR(m.median(), 5.0, 1e-12);
  m.add(2.0);
  CHECK_NEAR(m.median(), 3.5, 1e-12);
  CHECK_EQ(m.size(), std::size_t(4));

#ifdef HIDDEN
  RunningMedian e;
  CHECK(std::isnan(e.median()));
  CHECK_EQ(e.size(), std::size_t(0));
  // duplicates and negatives
  RunningMedian d;
  for (double x : {-3.0, -3.0, -3.0, 7.0}) d.add(x);
  CHECK_NEAR(d.median(), -3.0, 1e-12);
  // monotone increasing and decreasing streams
  RunningMedian inc, dec;
  for (int i = 1; i <= 1001; ++i) { inc.add(i); dec.add(-i); }
  CHECK_NEAR(inc.median(), 501.0, 1e-12);
  CHECK_NEAR(dec.median(), -501.0, 1e-12);
  // random stream vs brute force at checkpoints
  std::mt19937 g(7);
  std::normal_distribution<double> n(10.0, 2.0);
  RunningMedian r;
  std::vector<double> all;
  bool ok = true;
  for (int i = 0; i < 20000; ++i) {
    double x = (i % 50 == 0) ? 1e4 : n(g);
    r.add(x);
    all.push_back(x);
    if (i % 997 == 0 || i == 19999) ok = ok && std::abs(r.median() - bruteMedian(all)) < 1e-12;
  }
  CHECK(ok);
  CHECK_EQ(r.size(), std::size_t(20000));
#endif
  return TEST_SUMMARY();
}
