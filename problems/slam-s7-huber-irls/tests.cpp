// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

// y = m x + c with Gaussian noise; every 10th point is an outlier pushed UP by 15..40.
static std::vector<Eigen::Vector2d> wall(double m, double c, int n, double sigma, int outlier_every,
                                         unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> g(0.0, sigma);
  std::uniform_real_distribution<double> u(15.0, 40.0);
  std::vector<Eigen::Vector2d> pts;
  for (int i = 0; i < n; ++i) {
    const double x = 10.0 * i / n;
    double y = m * x + c + g(rng);
    if (outlier_every > 0 && i % outlier_every == 3) y += u(rng);
    pts.emplace_back(x, y);
  }
  return pts;
}

int main() {
  CHECK_NEAR(huberWeight(0.5, 1.0), 1.0, 1e-15);
  CHECK_NEAR(huberWeight(-4.0, 1.0), 0.25, 1e-15);
  CHECK_NEAR(madSigma({1, 2, 3, 4, 100}), 1.4826, 1e-12);  // median 3, |dev| = {2,1,0,1,97} -> 1

  auto pts = wall(2.0, 1.0, 200, 0.1, 10, 5);
  const Eigen::Vector2d ls = fitLineLS(pts);
  const Eigen::Vector2d hub = fitLineHuber(pts, 0.2);
  CHECK(std::abs(ls[1] - 1.0) > 1.0);            // LS is dragged by the outliers
  CHECK_NEAR(hub[0], 2.0, 0.03);
  CHECK_NEAR(hub[1], 1.0, 0.15);
  CHECK((hub - Eigen::Vector2d(2, 1)).norm() < 0.1 * (ls - Eigen::Vector2d(2, 1)).norm());

#ifdef HIDDEN
  // Exact data: LS exact, Huber equals LS and all weights 1.
  auto clean = wall(-0.5, 3.0, 50, 1e-12, 0, 1);
  CHECK_NEAR(fitLineLS(clean)[0], -0.5, 1e-9);
  CHECK_NEAR(fitLineLS(clean)[1], 3.0, 1e-9);
  std::vector<double> w;
  const Eigen::Vector2d hc = fitLineHuber(clean, 0.1, 20, &w);
  CHECK_NEAR(hc[0], -0.5, 1e-9);
  CHECK(w.size() == clean.size());
  CHECK(!w.empty() && *std::min_element(w.begin(), w.end()) == 1.0);

  // Weights flag the outliers.
  fitLineHuber(pts, 0.2, 30, &w);
  CHECK(w.size() == pts.size());
  if (w.size() == pts.size()) {
    bool ok = true;
    for (std::size_t i = 0; i < w.size(); ++i)
      ok = ok && (i % 10 == 3 ? w[i] < 0.05 : w[i] > 0.3);
    CHECK(ok);
  }

  // MAD-based threshold works too; negative-slope wall, downward outliers.
  auto neg = wall(-1.5, -2.0, 300, 0.05, 10, 9);
  for (auto& p : neg) if (p.y() > -1.5 * p.x() - 2.0 + 1.0) p.y() = -1.5 * p.x() - 2.0 - (p.y() - (-1.5 * p.x() - 2.0));
  std::vector<double> r0;
  const Eigen::Vector2d l0 = fitLineLS(neg);
  for (auto& p : neg) r0.push_back(p.y() - (l0[0] * p.x() + l0[1]));
  const double k = 1.345 * madSigma(r0);
  CHECK(k > 0.0);
  const Eigen::Vector2d hn = fitLineHuber(neg, k, 50);
  CHECK_NEAR(hn[0], -1.5, 0.03);
  CHECK_NEAR(hn[1], -2.0, 0.15);

  // MAD with even count.
  CHECK_NEAR(madSigma({1, 3}), 1.4826, 1e-12);  // median 2, |dev| = {1,1}
  CHECK_NEAR(madSigma({}), 0.0, 0.0);

  // Degenerate inputs.
  CHECK_MAT_NEAR(fitLineLS({}), Eigen::Vector2d::Zero(), 0.0);
  CHECK_MAT_NEAR(fitLineHuber({Eigen::Vector2d(1, 1)}, 1.0), Eigen::Vector2d::Zero(), 0.0);
  std::vector<Eigen::Vector2d> vertical = {{2, 0}, {2, 1}, {2, 5}};
  CHECK_MAT_NEAR(fitLineLS(vertical), Eigen::Vector2d::Zero(), 0.0);
  CHECK_MAT_NEAR(fitLineHuber(vertical, 1.0, 20, &w), Eigen::Vector2d::Zero(), 0.0);
  CHECK(w.size() == 3);
  // Two points: exact line through them.
  CHECK_MAT_NEAR(fitLineLS({Eigen::Vector2d(0, 1), Eigen::Vector2d(2, 5)}), Eigen::Vector2d(2, 1), 1e-12);
#endif
  return TEST_SUMMARY();
}
