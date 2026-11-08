// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <numeric>
#include "common/check.h"
#include "common/cloud_utils.h"

static std::vector<int> brute(const cu::Cloud& pts, const Eigen::Vector3d& q, int k) {
  std::vector<int> idx(pts.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) { return (pts[a] - q).squaredNorm() < (pts[b] - q).squaredNorm(); });
  idx.resize(std::max(0, std::min<int>(k, (int)pts.size())));
  return idx;
}

int main() {
  cu::Cloud line;
  for (int i = 0; i < 10; ++i) line.push_back(Eigen::Vector3d(i, 0, 0));
  std::vector<int> r = knn(line, Eigen::Vector3d(3.2, 0, 0), 3);
  CHECK(r == std::vector<int>({3, 4, 2}));

  const cu::Cloud c = cu::uniformCube(2000, 5.0);
  bool ok = true;
  for (int t = 0; t < 10; ++t) {
    Eigen::Vector3d q = Eigen::Vector3d::Random() * 5;
    ok &= knn(c, q, 8) == brute(c, q, 8);
  }
  CHECK(ok);
  CHECK(knn(c, Eigen::Vector3d::Zero(), 1).size() == 1);
  CHECK(knn(c, c[17], 1) == std::vector<int>({17}));

#ifdef HIDDEN
  // k > N
  CHECK(knn(line, Eigen::Vector3d(-1, 0, 0), 50) == std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
  // k <= 0, empty cloud
  CHECK(knn(line, Eigen::Vector3d::Zero(), 0).empty());
  CHECK(knn(line, Eigen::Vector3d::Zero(), -3).empty());
  CHECK(knn({}, Eigen::Vector3d::Zero(), 5).empty());
  // ties: equidistant points -> smaller index first; duplicates
  cu::Cloud tie = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, 0, 5}, {1, 0, 0}};
  CHECK(knn(tie, Eigen::Vector3d::Zero(), 4) == std::vector<int>({0, 1, 2, 4}));
  // single point
  CHECK(knn(cu::Cloud{{2, 2, 2}}, Eigen::Vector3d::Zero(), 3) == std::vector<int>({0}));
  // large cloud
  const cu::Cloud big = cu::uniformCube(100000, 50.0);
  const Eigen::Vector3d q(1, -2, 3);
  CHECK(knn(big, q, 10) == brute(big, q, 10));
  CHECK(knn(big, q, 1).size() == 1);
#endif
  return TEST_SUMMARY();
}
