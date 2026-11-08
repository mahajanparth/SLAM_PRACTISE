// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

static bool same(const std::vector<Interval>& a, const std::vector<Interval>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (std::abs(a[i].start - b[i].start) > 1e-12 || std::abs(a[i].end - b[i].end) > 1e-12) return false;
  return true;
}

int main() {
  auto m = mergeDropouts({{5, 6}, {1, 3}, {2, 4}, {4.05, 4.5}}, 0.1);
  CHECK(same(m, {{1, 4.5}, {5, 6}}));
  CHECK(same(mergeDropouts({{0, 1}, {1, 2}}, 0.0), {{0, 2}}));      // touching
  CHECK(same(mergeDropouts({{0, 1}, {1.5, 2}}, 0.0), {{0, 1}, {1.5, 2}}));

  CHECK(isDown(m, 1.0));
  CHECK(!isDown(m, 4.5));   // half-open
  CHECK(!isDown(m, 0.5));
  CHECK_NEAR(uptime(m, 0.0, 10.0), 10.0 - 3.5 - 1.0, 1e-12);

  auto both = bothDown({{0, 5}, {8, 12}}, {{3, 9}, {11, 20}});
  CHECK(same(both, {{3, 5}, {8, 9}, {11, 12}}));

#ifdef HIDDEN
  CHECK(mergeDropouts({}, 1.0).empty());
  CHECK(same(mergeDropouts({{2, 3}}, 0.0), {{2, 3}}));
  CHECK(same(mergeDropouts({{0, 10}, {2, 3}, {4, 5}}, 0.0), {{0, 10}}));        // nested
  CHECK(same(mergeDropouts({{-5, -3}, {-3.05, -1}}, 0.0), {{-5, -1}}));          // negative times
  CHECK(same(mergeDropouts({{1, 1}, {3, 4}}, 0.5), {{1, 1}, {3, 4}}));            // zero-length kept
  CHECK(same(mergeDropouts({{1, 1}, {1, 4}}, 0.0), {{1, 4}}));
  CHECK(!isDown({}, 1.0));
  CHECK(!isDown({{1, 1}}, 1.0));
  CHECK(isDown(m, 5.0) && isDown(m, 5.999) && !isDown(m, 6.0) && !isDown(m, 100.0));
  CHECK_NEAR(uptime(m, 2.0, 5.5), 0.5, 1e-12);       // [4.5,5) up
  CHECK_NEAR(uptime(m, 3.0, 3.0), 0.0, 1e-15);
  CHECK_NEAR(uptime(m, 5.0, 2.0), 0.0, 1e-15);
  CHECK_NEAR(uptime({}, 0.0, 7.0), 7.0, 1e-15);
  CHECK(bothDown({}, m).empty());
  CHECK(bothDown({{0, 1}}, {{1, 2}}).empty());       // touching -> empty overlap
  CHECK(same(bothDown({{0, 100}}, {{1, 2}, {3, 4}, {50, 150}}), {{1, 2}, {3, 4}, {50, 100}}));
  // random vs sampled brute force
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> S(0, 100), L(0, 3);
  std::vector<Interval> raw;
  for (int i = 0; i < 300; ++i) { double s = S(rng); raw.push_back({s, s + L(rng)}); }
  auto mm = mergeDropouts(raw, 0.0);
  bool ok = true;
  for (std::size_t i = 1; i < mm.size(); ++i) ok = ok && mm[i - 1].end < mm[i].start;
  for (int k = 0; k < 2000 && ok; ++k) {
    double t = k * 0.0517;
    bool brute = false;
    for (auto& iv : raw) brute = brute || (t >= iv.start && t < iv.end);
    ok = isDown(mm, t) == brute;
  }
  CHECK(ok);
  double cov = 0;
  for (auto& iv : mm) cov += iv.end - iv.start;
  CHECK_NEAR(uptime(mm, -10, 200), 210 - cov, 1e-9);
#endif
  return TEST_SUMMARY();
}
