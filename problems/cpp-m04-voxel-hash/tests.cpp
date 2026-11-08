// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <unordered_set>

int main() {
  CHECK((VoxelKey{1, 2, 3} == VoxelKey{1, 2, 3}));
  CHECK((!(VoxelKey{1, 2, 3} == VoxelKey{3, 2, 1})));
  VoxelKey k = toKey({1.2, -0.3, 0.0}, 0.5);
  CHECK(k.x == 2 && k.y == -1 && k.z == 0);  // floor, not truncation

  std::vector<Point3> c = {{0.1, 0.1, 0.1}, {5.5, 0, 0}, {0.3, 0.3, 0.3}, {5.7, 0.2, 0.4}};
  auto d = voxelDownsample(c, 1.0);
  CHECK(d.size() == 2);
  if (d.size() == 2) {
    CHECK_NEAR(d[0].x, 0.2, 1e-12);  // first-appearance order
    CHECK_NEAR(d[1].x, 5.6, 1e-12);
    CHECK_NEAR(d[1].z, 0.2, 1e-12);
  }

  // hash quality: a 40^3 block of keys around the origin
  std::unordered_set<std::size_t> hs;
  VoxelKeyHash H;
  for (int x = -20; x < 20; ++x)
    for (int y = -20; y < 20; ++y)
      for (int z = -20; z < 20; ++z) hs.insert(H(VoxelKey{x, y, z}));
  CHECK(hs.size() >= 32000);

#ifdef HIDDEN
  CHECK((H(VoxelKey{4, -7, 9}) == H(VoxelKey{4, -7, 9})));  // deterministic / consistent with ==
  CHECK((H(VoxelKey{1, 2, 3}) != H(VoxelKey{3, 2, 1})));    // permutations differ
  // negative boundaries: exactly on a boundary belongs to the upper voxel
  VoxelKey b = toKey({-1.0, -0.0001, 2.0}, 1.0);
  CHECK(b.x == -1 && b.y == -1 && b.z == 2);
  // points straddling 0 must NOT be merged
  CHECK(voxelDownsample({{-0.1, 0, 0}, {0.1, 0, 0}}, 1.0).size() == 2);
  // empty and single
  CHECK(voxelDownsample({}, 0.2).empty());
  auto one = voxelDownsample({{-3.3, 4.4, -5.5}}, 0.1);
  CHECK(one.size() == 1 && std::abs(one[0].y - 4.4) < 1e-12);
  // large: 200k points on a 0.1 m lattice -> voxel 1.0 merges 10^3 per cell
  std::vector<Point3> big;
  big.reserve(200000);
  for (int i = 0; i < 200000; ++i) {
    int a = i % 100, bb = (i / 100) % 40, cc = i / 4000;  // 100 x 40 x 50 lattice
    big.push_back({a * 0.1 - 5.0 + 0.05, bb * 0.1 - 2.0 + 0.05, cc * 0.1 - 2.0 + 0.05});
  }
  auto bd = voxelDownsample(big, 1.0);
  CHECK(bd.size() == 10 * 4 * 5);
  if (!bd.empty()) { CHECK_NEAR(bd[0].x, -4.5, 1e-9); CHECK_NEAR(bd[0].z, -1.5, 1e-9); }
  // every centroid lies inside its own voxel
  bool inside = true;
  for (const auto& p : bd) {
    VoxelKey kk = toKey(p, 1.0);
    inside &= (p.x >= kk.x && p.x < kk.x + 1 && p.y >= kk.y && p.y < kk.y + 1);
  }
  CHECK(inside);
#endif
  return TEST_SUMMARY();
}
