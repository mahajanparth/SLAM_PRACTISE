// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  FrameDiff d = diffFrames({5, 3, 9, 3, 1}, {9, 2, 5, 5, 7});
  CHECK(d.tracked == (std::vector<int>{5, 9}));
  CHECK(d.lost == (std::vector<int>{1, 3}));
  CHECK(d.appeared == (std::vector<int>{2, 7}));

  std::vector<Point3> c = {{0.1, 0.1, 0.1}, {0.2, 0.3, 0.4}, {1.5, 0, 0}, {-0.1, 0, 0}};
  CHECK_EQ(countUniqueVoxels(c, 1.0), std::size_t(3));  // -0.1 is voxel -1

  CHECK(hasDuplicateIds({4, 8, 15, 16, 23, 4}));
  CHECK(!hasDuplicateIds({4, 8, 15, 16, 23, 42}));

#ifdef HIDDEN
  // empty frames
  FrameDiff e = diffFrames({}, {});
  CHECK(e.tracked.empty() && e.lost.empty() && e.appeared.empty());
  FrameDiff allNew = diffFrames({}, {3, 3, -1});
  CHECK(allNew.appeared == (std::vector<int>{-1, 3}));
  FrameDiff allLost = diffFrames({2, 2, 2}, {});
  CHECK(allLost.lost == (std::vector<int>{2}) && allLost.tracked.empty());
  // identical frames (different order, duplicates)
  FrameDiff same = diffFrames({1, 2, 3}, {3, 3, 2, 1});
  CHECK(same.tracked == (std::vector<int>{1, 2, 3}) && same.lost.empty() && same.appeared.empty());
  CHECK(!hasDuplicateIds({}));
  CHECK(!hasDuplicateIds({7}));
  CHECK_EQ(countUniqueVoxels({}, 0.5), std::size_t(0));
  // large: 2e5 ids, half overlap
  std::vector<int> p, q;
  for (int i = 0; i < 100000; ++i) { p.push_back((i * 7) % 100000); q.push_back(50000 + (i * 3) % 100000); }
  FrameDiff big = diffFrames(p, q);
  CHECK(big.tracked.size() == 50000 && big.lost.size() == 50000 && big.appeared.size() == 50000);
  CHECK(!big.tracked.empty() && big.tracked.front() == 50000 && big.lost.back() == 49999);
  CHECK(!hasDuplicateIds(p));
  // big cloud on a 0.25 m lattice spanning negative coords, voxel 1.0 -> 4^3 points per voxel
  std::vector<Point3> cl;
  for (int x = -20; x < 20; ++x)
    for (int y = -20; y < 20; ++y)
      for (int z = -20; z < 20; ++z) cl.push_back({x * 0.25 + 0.1, y * 0.25 + 0.1, z * 0.25 + 0.1});
  CHECK_EQ(countUniqueVoxels(cl, 1.0), std::size_t(10 * 10 * 10));
#endif
  return TEST_SUMMARY();
}
