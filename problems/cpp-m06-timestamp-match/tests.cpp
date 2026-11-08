// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

using Matches = std::vector<std::pair<std::size_t, std::size_t>>;

static Matches brute(const std::vector<double>& l, const std::vector<double>& c, double max_dt) {
  Matches out;
  for (std::size_t i = 0; i < l.size(); ++i) {
    std::size_t best = 0; double bd = 1e300;
    for (std::size_t j = 0; j < c.size(); ++j)
      if (std::abs(c[j] - l[i]) < bd) { bd = std::abs(c[j] - l[i]); best = j; }
    if (!c.empty() && bd <= max_dt) out.emplace_back(i, best);
  }
  return out;
}

int main() {
  std::vector<double> lidar = {0.00, 0.10, 0.20, 0.30};
  std::vector<double> cam = {0.01, 0.04, 0.07, 0.11, 0.14, 0.17, 0.32};
  Matches m = matchTimestamps(lidar, cam, 0.025);
  // 0.00->0.01 (j0); 0.10->0.11 (j3); 0.20: nearest 0.17 is 0.03 away -> dropped; 0.30->0.32 (j6)
  CHECK(m == (Matches{{0, 0}, {1, 3}, {3, 6}}));

  std::vector<Point3> c = {{0, 0, -1.6}, {1, 0, 0.5}, {2, 0, -1.7}, {3, 0, 2.0}, {4, 0, -1.5}};
  std::size_t g = partitionGround(c, -1.0);
  CHECK_EQ(g, std::size_t(3));
  CHECK(c.size() == 5);
  if (c.size() == 5) {
    CHECK(c[0].x == 0 && c[1].x == 2 && c[2].x == 4);  // ground, original order
    CHECK(c[3].x == 1 && c[4].x == 3);                  // non-ground, original order
  }

#ifdef HIDDEN
  // empty inputs
  CHECK(matchTimestamps({}, cam, 1.0).empty());
  CHECK(matchTimestamps(lidar, {}, 1.0).empty());
  // before the first / after the last camera frame
  CHECK(matchTimestamps({-5.0, 99.0}, {1.0, 2.0}, 1e9) == (Matches{{0, 0}, {1, 1}}));
  // exact tie -> earlier camera; exact match
  CHECK(matchTimestamps({1.5, 2.0}, {1.0, 2.0, 3.0}, 0.5) == (Matches{{0, 0}, {1, 1}}));
  // duplicate camera stamps -> smallest index
  CHECK(matchTimestamps({2.1}, {1.0, 2.0, 2.0, 2.0, 5.0}, 1.0) == (Matches{{0, 1}}));
  CHECK(matchTimestamps({1.9}, {2.0, 2.0}, 1.0) == (Matches{{0, 0}}));
  // max_dt boundary inclusive (exactly representable values)
  CHECK(matchTimestamps({1.0}, {1.5}, 0.5) == (Matches{{0, 0}}));
  CHECK(matchTimestamps({1.0}, {1.5}, 0.25).empty());
  // large randomised-but-deterministic vs brute force (subset of lidar to keep brute force cheap)
  std::vector<double> bigCam, bigLidar;
  for (int j = 0; j < 30000; ++j) bigCam.push_back(j * (1.0 / 30) + ((j * 7919) % 11) * 1e-3);
  std::sort(bigCam.begin(), bigCam.end());
  for (int i = 0; i < 2000; ++i) bigLidar.push_back(i * 0.5 + ((i * 104729) % 17) * 1e-3);
  Matches bm = matchTimestamps(bigLidar, bigCam, 0.012);
  std::vector<double> sub(bigLidar.begin(), bigLidar.begin() + 300);
  Matches bsub = brute(sub, bigCam, 0.012);
  Matches head;
  for (auto& pr : bm) if (pr.first < 300) head.push_back(pr);
  CHECK(head == bsub);
  CHECK(!bm.empty() && bm.size() <= bigLidar.size());
  // partition: none / all ground, empty, large stable
  std::vector<Point3> none = {{0, 0, 1}, {1, 0, 2}};
  CHECK_EQ(partitionGround(none, 0.0), std::size_t(0));
  CHECK(none[0].x == 0 && none[1].x == 1);
  CHECK_EQ(partitionGround(none, 10.0), std::size_t(2));
  std::vector<Point3> empty;
  CHECK_EQ(partitionGround(empty, 0.0), std::size_t(0));
  std::vector<Point3> big;
  for (int i = 0; i < 200000; ++i) big.push_back({double(i), 0, (i % 3 == 0) ? -2.0 : 1.0});
  std::size_t bg = partitionGround(big, -1.0);
  CHECK_EQ(bg, std::size_t(66667));
  bool ok = true;
  for (std::size_t i = 1; i < big.size(); ++i)
    if (i != bg) ok &= big[i].x > big[i - 1].x;
  CHECK(ok);
#endif
  return TEST_SUMMARY();
}
