#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

// Given: brute-force kNN, sorted by distance, ties -> smaller index.
std::vector<int> knn(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector3d& q, int k) {
  if (k <= 0 || pts.empty()) return {};
  const int n = static_cast<int>(pts.size()), kk = std::min(k, n);
  std::vector<std::pair<double, int>> d(n);
  for (int i = 0; i < n; ++i) d[i] = {(pts[i] - q).squaredNorm(), i};
  std::partial_sort(d.begin(), d.begin() + kk, d.end());
  std::vector<int> out(kk);
  for (int i = 0; i < kk; ++i) out[i] = d[i].second;
  return out;
}

std::vector<int> statisticalOutlierRemoval(const std::vector<Eigen::Vector3d>& pts, int k, double alpha) {
  const int n = static_cast<int>(pts.size());
  std::vector<int> all(n);
  std::iota(all.begin(), all.end(), 0);
  const int kk = std::min(k, n - 1);
  if (n <= 1 || kk <= 0) return all;

  // Pass 1: mean distance to the k nearest OTHER points.
  std::vector<double> mean_d(n);
  for (int i = 0; i < n; ++i) {
    const std::vector<int> nb = knn(pts, pts[i], kk + 1);
    double sum = 0.0;
    int cnt = 0;
    for (int j : nb) {
      if (j == i || cnt == kk) continue;  // skip self by INDEX (duplicates still count)
      sum += (pts[j] - pts[i]).norm();
      ++cnt;
    }
    mean_d[i] = sum / cnt;
  }

  // Pass 2: global statistics (population std).
  const double mu = std::accumulate(mean_d.begin(), mean_d.end(), 0.0) / n;
  double var = 0.0;
  for (double d : mean_d) var += (d - mu) * (d - mu);
  const double sigma = std::sqrt(var / n);
  const double thr = mu + alpha * sigma;

  std::vector<int> keep;
  keep.reserve(n);
  for (int i = 0; i < n; ++i)
    if (mean_d[i] <= thr) keep.push_back(i);  // <= keeps everything when sigma == 0
  return keep;
}
