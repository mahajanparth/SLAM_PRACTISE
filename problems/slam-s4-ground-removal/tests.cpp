// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  // flat ground 1.8 m below the sensor + a 1x1x1 box standing on it at x in [4,5]
  cu::Cloud pts;
  for (int i = 0; i < 20; ++i)
    for (int j = 0; j < 20; ++j) pts.push_back(Eigen::Vector3d(-5 + 0.5 * i + 0.01, -5 + 0.5 * j + 0.01, -1.8 + 0.01 * ((i + j) % 3)));
  const int n_ground = int(pts.size());
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) pts.push_back(Eigen::Vector3d(4.1 + 0.2 * i, 0.1 + 0.2 * j, -1.5 + 0.2 * (i + j) / 2.0));
  std::vector<int> expect(pts.size() - n_ground);
  for (size_t i = 0; i < expect.size(); ++i) expect[i] = n_ground + int(i);

  CHECK(removeGroundHeight(pts, 1.8, 0.2) == expect);
  CHECK(removeGroundGrid(pts, 1.0, 0.2) == std::vector<int>(expect.begin(), expect.end()));
  CHECK(removeGroundHeight({}, 1.8, 0.2).empty());
  CHECK(removeGroundGrid({}, 1.0, 0.2).empty());

#ifdef HIDDEN
  // 10% slope: ground z = -1.8 + 0.1 x for x in [-20, 20]. The height threshold fails far away; the grid copes.
  cu::Cloud slope;
  for (int i = 0; i < 81; ++i)
    for (int j = 0; j < 11; ++j) {
      double x = -20 + 0.5 * i + 0.013, y = -2.5 + 0.5 * j + 0.017;
      slope.push_back(Eigen::Vector3d(x, y, -1.8 + 0.1 * x));
    }
  const int ns = int(slope.size());
  slope.push_back(Eigen::Vector3d(15.2, 0.3, -1.8 + 1.52 + 1.0));  // pole, 1 m above the sloped ground
  slope.push_back(Eigen::Vector3d(15.3, 0.3, -1.8 + 1.53 + 0.5));
  auto hs = removeGroundHeight(slope, 1.8, 0.2);
  CHECK(hs.size() > 100);  // naive: uphill ground counted as obstacle
  CHECK(removeGroundGrid(slope, 0.5, 0.2) == std::vector<int>({ns, ns + 1}));
  // negative coordinates: floor cells. A low point at x=+0.1 must not be the minimum for x=-0.1.
  cu::Cloud nc = {{-0.1, 0.2, 0.0}, {0.1, 0.2, -1.0}};
  CHECK(removeGroundGrid(nc, 1.0, 0.2).empty());  // each point is the minimum of its own cell
  cu::Cloud nc2 = {{-0.1, -0.2, 0.0}, {-0.9, -0.8, -1.0}};
  CHECK(removeGroundGrid(nc2, 1.0, 0.2) == std::vector<int>({0}));  // same cell (-1,-1)
  // cell <= 0 throws
  bool threw = false;
  try { removeGroundGrid(nc, 0.0, 0.2); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  // threshold boundary: exactly at the limit is NOT ground (strict <)
  CHECK(removeGroundHeight({Eigen::Vector3d(0, 0, -1.5)}, 1.75, 0.25) == std::vector<int>({0}));
  // larger random scene keeps input order
  cu::Cloud big = cu::planePoints(20000, Eigen::Vector3d::UnitZ(), 1.8, 30.0, 0.02);
  for (int i = 0; i < 100; ++i) big.push_back(Eigen::Vector3d(0.1 * i - 5, 3.0, 0.5));
  auto kb = removeGroundGrid(big, 1.0, 0.3);
  CHECK(std::is_sorted(kb.begin(), kb.end()));
  CHECK(kb.size() == 100 && kb.front() == 20000);
#endif
  return TEST_SUMMARY();
}
