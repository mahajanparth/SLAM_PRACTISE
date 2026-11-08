// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include "common/check.h"
#include "common/cloud_utils.h"

static bool contains(const std::vector<int>& v, int x) { return std::find(v.begin(), v.end(), x) != v.end(); }

int main() {
  // dense cube + a few far isolated points
  cu::Cloud c = cu::uniformCube(600, 1.0);
  const int n0 = int(c.size());
  c.push_back({8, 8, 8});
  c.push_back({-9, 0, 3});
  c.push_back({0, 12, -7});
  auto in = statisticalOutlierRemoval(c, 8, 1.0);
  CHECK(!contains(in, n0) && !contains(in, n0 + 1) && !contains(in, n0 + 2));
  CHECK(in.size() > 500 && in.size() < c.size());
  CHECK(std::is_sorted(in.begin(), in.end()));

  // hand-checkable: points on a line at x = 0,1,2,3 and one at 10, k = 1
  //   mean NN distances: 1,1,1,1,7 -> mu = 2.2, sigma = 2.4 -> thr(alpha=1) = 4.6
  cu::Cloud line = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {10, 0, 0}};
  CHECK(statisticalOutlierRemoval(line, 1, 1.0) == std::vector<int>({0, 1, 2, 3}));
  // large alpha keeps everything
  CHECK(statisticalOutlierRemoval(line, 1, 10.0) == std::vector<int>({0, 1, 2, 3, 4}));

#ifdef HIDDEN
  // all identical points: sigma = 0 -> keep all
  CHECK(statisticalOutlierRemoval(cu::Cloud(10, Eigen::Vector3d(1, 2, 3)), 3, 1.0).size() == 10);
  // empty, single, k <= 0
  CHECK(statisticalOutlierRemoval({}, 5, 1.0).empty());
  CHECK(statisticalOutlierRemoval(cu::Cloud{{1, 1, 1}}, 5, 1.0) == std::vector<int>({0}));
  CHECK(statisticalOutlierRemoval(line, 0, 1.0).size() == 5);
  // k >= N clamps to N-1: line with k = 10 -> mean distances to all others
  //   x=0: (1+2+3+10)/4=4, x=1: (1+1+2+9)/4=3.25, x=2: (2+1+1+8)/4=3, x=3: (3+2+1+7)/4=3.25, x=10: (10+9+8+7)/4=8.5
  //   mu = 4.4, sigma = sqrt(((.4)^2+(1.15)^2+(1.4)^2+(1.15)^2+(4.1)^2)/5) = 2.064... -> thr(0) = 4.4
  CHECK(statisticalOutlierRemoval(line, 10, 0.0) == std::vector<int>({0, 1, 2, 3}));
  // duplicates: the self-skip must be by index (a duplicate at distance 0 is a real neighbour)
  cu::Cloud dup = {{0, 0, 0}, {0, 0, 0}, {5, 0, 0}, {5, 0, 0}, {50, 0, 0}};
  // k=1: mean d = 0,0,0,0,45 -> mu = 9, sigma = 18 -> thr(0.5) = 18 -> drop index 4
  CHECK(statisticalOutlierRemoval(dup, 1, 0.5) == std::vector<int>({0, 1, 2, 3}));
  // negative alpha: only points below the mean survive
  auto neg = statisticalOutlierRemoval(line, 1, -0.1);
  CHECK(neg == std::vector<int>({0, 1, 2, 3}));
  // noisy plane with sparse outliers above it
  cu::Cloud pl = cu::planePoints(1500, Eigen::Vector3d::UnitZ(), 0.0, 3.0, 0.01);
  const int base = int(pl.size());
  for (int i = 0; i < 10; ++i) pl.push_back(Eigen::Vector3d(-3 + 0.6 * i, 0.5 * i - 2, 2.0 + 0.3 * i));
  auto ip = statisticalOutlierRemoval(pl, 10, 2.0);
  bool removed = true;
  for (int i = 0; i < 10; ++i) removed &= !contains(ip, base + i);
  CHECK(removed);
  CHECK(ip.size() > 1400);
#endif
  return TEST_SUMMARY();
}
