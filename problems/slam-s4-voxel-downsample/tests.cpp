// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <array>
#include <map>
#include "common/check.h"
#include "common/cloud_utils.h"

static bool lexLess(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3);
}
// Brute-force reference with std::map (ordered, slow, obviously correct). Uses division like the spec.
static std::vector<Eigen::Vector3d> reference(const std::vector<Eigen::Vector3d>& pts, double leaf) {
  std::map<std::array<long long, 3>, std::pair<Eigen::Vector3d, int>> m;
  for (const auto& p : pts) {
    std::array<long long, 3> k{(long long)std::floor(p.x() / leaf), (long long)std::floor(p.y() / leaf),
                               (long long)std::floor(p.z() / leaf)};
    auto it = m.find(k);
    if (it == m.end()) m.emplace(k, std::make_pair(p, 1));
    else { it->second.first += p; ++it->second.second; }
  }
  std::vector<Eigen::Vector3d> out;
  for (auto& kv : m) out.push_back(kv.second.first / kv.second.second);
  return out;
}
static bool sameSet(std::vector<Eigen::Vector3d> a, std::vector<Eigen::Vector3d> b, double tol) {
  if (a.size() != b.size()) return false;
  std::sort(a.begin(), a.end(), lexLess);
  std::sort(b.begin(), b.end(), lexLess);
  for (size_t i = 0; i < a.size(); ++i)
    if ((a[i] - b[i]).norm() > tol) return false;
  return true;
}

int main() {
  // two points in one voxel, one in another
  std::vector<Eigen::Vector3d> pts = {{0.1, 0.1, 0.1}, {0.3, 0.3, 0.3}, {1.5, 0.2, 0.2}};
  auto out = voxelDownsample(pts, 1.0);
  CHECK_EQ(out.size(), size_t(2));
  CHECK(sameSet(out, {{0.2, 0.2, 0.2}, {1.5, 0.2, 0.2}}, 1e-12));

  // random cloud, compare to the brute-force reference (grid aligned to avoid boundary ambiguity)
  cu::Cloud c = cu::uniformCube(5000, 3.0);
  for (auto& p : c) p += Eigen::Vector3d(0.0123, 0.0456, 0.0789);
  auto got = voxelDownsample(c, 0.5);
  auto ref = reference(c, 0.5);
  CHECK_EQ(got.size(), ref.size());
  CHECK(sameSet(got, ref, 1e-9));
  CHECK(voxelDownsample({}, 0.1).empty());

#ifdef HIDDEN
  // negative coordinates: -0.01 and 0.01 are in different voxels (floor, not truncation)
  auto neg = voxelDownsample({{-0.01, 0.5, 0.5}, {0.01, 0.5, 0.5}}, 1.0);
  CHECK_EQ(neg.size(), size_t(2));
  auto neg2 = voxelDownsample({{-0.2, -0.2, -0.2}, {-0.8, -0.6, -0.4}}, 1.0);  // same voxel (-1,-1,-1)
  CHECK_EQ(neg2.size(), size_t(1));
  if (neg2.size() == 1) CHECK_MAT_NEAR(neg2[0], Eigen::Vector3d(-0.5, -0.4, -0.3), 1e-12);
  // all points in one voxel
  cu::Cloud tight = cu::uniformCube(1000, 0.04);
  for (auto& p : tight) p += Eigen::Vector3d(0.05, 0.05, 0.05);
  auto one = voxelDownsample(tight, 0.1);
  CHECK_EQ(one.size(), size_t(1));
  // leaf <= 0 throws
  bool threw = false;
  try { voxelDownsample(pts, 0.0); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { voxelDownsample(pts, -1.0); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  // far from the origin, with symmetric keys that defeat a naive x^y^z hash
  std::vector<Eigen::Vector3d> far;
  for (int i = -20; i < 20; ++i)
    for (int j = -20; j < 20; ++j) far.push_back(Eigen::Vector3d(9000.05 + i * 0.1, -9000.05 + j * 0.1, 3.05));
  auto gf = voxelDownsample(far, 0.1);
  CHECK(sameSet(gf, reference(far, 0.1), 1e-6));
  // large cloud: 150k points
  cu::Cloud big = cu::uniformCube(150000, 20.0);
  auto gb = voxelDownsample(big, 1.0);
  CHECK(sameSet(gb, reference(big, 1.0), 1e-9));
#endif
  return TEST_SUMMARY();
}
