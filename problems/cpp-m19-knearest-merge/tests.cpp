// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

static std::vector<int> bruteKnn(const std::vector<Point3>& c, const Point3& q, int k) {
  std::vector<std::pair<double, int>> d;
  for (std::size_t i = 0; i < c.size(); ++i) {
    double dx = c[i].x - q.x, dy = c[i].y - q.y, dz = c[i].z - q.z;
    d.push_back({dx * dx + dy * dy + dz * dz, (int)i});
  }
  std::sort(d.begin(), d.end());
  std::vector<int> out;
  for (int i = 0; i < k && i < (int)d.size(); ++i) out.push_back(d[i].second);
  return out;
}

int main() {
  std::vector<Point3> cloud = {{5, 0, 0}, {1, 0, 0}, {0, 3, 0}, {0, 0, 2}, {-1, 0, 0}};
  CHECK(kNearest(cloud, {0, 0, 0}, 3) == (std::vector<int>{1, 4, 3}));  // tie 1 vs 4 -> smaller index first
  CHECK(kNearest(cloud, {5, 0, 0}, 1) == (std::vector<int>{0}));
  CHECK(kNearest(cloud, {0, 0, 0}, 0).empty());

  std::vector<std::vector<Stamped>> streams = {
      {{0.0, 0}, {0.2, 0}, {0.4, 0}},  // lidar
      {{0.1, 1}, {0.2, 1}},            // camera
      {{0.05, 2}}};                    // gps
  auto m = mergeStreams(streams);
  CHECK_EQ(m.size(), 6u);
  if (m.size() == 6) {
    CHECK(m[1].t == 0.05 && m[1].sensor == 2);
    CHECK(m[3].t == 0.2 && m[3].sensor == 0);  // tie -> lower stream index first
    CHECK(m[4].sensor == 1 && m[5].t == 0.4);
  }

#ifdef HIDDEN
  CHECK(kNearest({}, {0, 0, 0}, 3).empty());
  CHECK(kNearest(cloud, {0, 0, 0}, 100) == (std::vector<int>{1, 4, 3, 2, 0}));
  CHECK(kNearest(cloud, {0, 0, 0}, -2).empty());
  std::vector<Point3> same(10, Point3{1, 1, 1});
  CHECK(kNearest(same, {0, 0, 0}, 4) == (std::vector<int>{0, 1, 2, 3}));
  // random cloud vs brute force
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> U(-10, 10);
  std::vector<Point3> big(50000);
  for (auto& p : big) p = {U(rng), U(rng), U(rng)};
  Point3 q{1.5, -2.0, 0.3};
  CHECK(kNearest(big, q, 16) == bruteKnn(big, q, 16));
  CHECK(kNearest(big, q, 1) == bruteKnn(big, q, 1));
  // merge edge cases
  CHECK(mergeStreams({}).empty());
  CHECK(mergeStreams({{}, {}, {}}).empty());
  auto one = mergeStreams({{}, {{1.0, 7}, {1.0, 8}}, {}});
  CHECK(one.size() == 2 && one[0].sensor == 7 && one[1].sensor == 8);
  // many streams vs sort
  std::vector<std::vector<Stamped>> many(20);
  std::uniform_int_distribution<int> T(0, 500);
  std::size_t total = 0;
  for (int s = 0; s < 20; ++s) {
    std::vector<double> ts(s * 37 % 200);
    for (auto& t : ts) t = T(rng) * 0.01;
    std::sort(ts.begin(), ts.end());
    for (double t : ts) many[s].push_back({t, s});
    total += ts.size();
  }
  auto mm = mergeStreams(many);
  bool ok = mm.size() == total;
  for (std::size_t i = 1; ok && i < mm.size(); ++i)
    ok = mm[i - 1].t < mm[i].t || (mm[i - 1].t == mm[i].t && mm[i - 1].sensor <= mm[i].sensor);
  CHECK(ok);
#endif
  return TEST_SUMMARY();
}
