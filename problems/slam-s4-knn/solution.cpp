#include <Eigen/Dense>
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

// O(N log k): partial_sort only keeps the k smallest in order.
// (d2, index) pairs compare lexicographically, so ties are broken by smaller index.
std::vector<int> knn(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector3d& q, int k) {
  if (k <= 0 || pts.empty()) return {};
  const int n = static_cast<int>(pts.size());
  const int kk = std::min(k, n);
  std::vector<std::pair<double, int>> d(n);
  for (int i = 0; i < n; ++i) d[i] = {(pts[i] - q).squaredNorm(), i};  // no sqrt
  std::partial_sort(d.begin(), d.begin() + kk, d.end());
  std::vector<int> out(kk);
  for (int i = 0; i < kk; ++i) out[i] = d[i].second;
  return out;
}
