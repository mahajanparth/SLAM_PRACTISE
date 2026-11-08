// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

static void makeData(double a, double b, double c, int n, double sigma, unsigned seed,
                     std::vector<double>& x, std::vector<double>& y) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> g(0.0, sigma > 0 ? sigma : 1.0);
  x.clear(); y.clear();
  for (int i = 0; i < n; ++i) {
    const double xi = double(i) / n;
    x.push_back(xi);
    y.push_back(std::exp(a * xi * xi + b * xi + c) + (sigma > 0 ? g(rng) : 0.0));
  }
}

// Gradient of 0.5*sum r^2 (first-order optimality check), independent of the candidate.
static Eigen::Vector3d gradient(const std::vector<double>& x, const std::vector<double>& y,
                                const Eigen::Vector3d& p) {
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double f = std::exp(p[0] * x[i] * x[i] + p[1] * x[i] + p[2]);
    g += Eigen::Vector3d(-f * x[i] * x[i], -f * x[i], -f) * (y[i] - f);
  }
  return g;
}

int main() {
  std::vector<double> x, y;
  // Small noise: parameters recovered closely.
  makeData(1.0, 2.0, 1.0, 100, 0.05, 7, x, y);
  GNResult r = fitExpQuadratic(x, y);
  CHECK(r.converged);
  CHECK(r.iterations > 0 && r.iterations < 20);
  CHECK_MAT_NEAR(r.params, Eigen::Vector3d(1.0, 2.0, 1.0), 0.05);
  CHECK(gradient(x, y, r.params).norm() < 1e-6);

  // Classic setup: noise sigma = 1 (as in "14 lectures on visual SLAM").
  makeData(1.0, 2.0, 1.0, 100, 1.0, 42, x, y);
  r = fitExpQuadratic(x, y);
  CHECK(r.converged);
  CHECK(r.iterations < 20);
  CHECK_MAT_NEAR(r.params, Eigen::Vector3d(1.0, 2.0, 1.0), 0.6);
  CHECK(gradient(x, y, r.params).norm() < 1e-6);

#ifdef HIDDEN
  // Noise-free data: exact recovery, cost ~ 0.
  makeData(1.0, 2.0, 1.0, 50, 0.0, 1, x, y);
  r = fitExpQuadratic(x, y);
  CHECK(r.converged);
  CHECK_MAT_NEAR(r.params, Eigen::Vector3d(1.0, 2.0, 1.0), 1e-6);
  CHECK_NEAR(r.final_cost, 0.0, 1e-12);

  // Negative curvature & offset.
  makeData(-0.5, 1.0, -0.3, 200, 0.01, 3, x, y);
  r = fitExpQuadratic(x, y);
  CHECK(r.converged);
  CHECK_MAT_NEAR(r.params, Eigen::Vector3d(-0.5, 1.0, -0.3), 0.05);

  // final_cost must correspond to the returned params.
  {
    double c = 0;
    for (std::size_t i = 0; i < x.size(); ++i) {
      const double f = std::exp(r.params[0] * x[i] * x[i] + r.params[1] * x[i] + r.params[2]);
      c += (y[i] - f) * (y[i] - f);
    }
    CHECK_NEAR(r.final_cost, 0.5 * c, 1e-9);
  }

  // Under-determined input: return init untouched.
  Eigen::Vector3d init(0.1, 0.2, 0.3);
  r = fitExpQuadratic({0.0, 1.0}, {1.0, 2.0}, init);
  CHECK(!r.converged);
  CHECK_EQ(r.iterations, 0);
  CHECK_MAT_NEAR(r.params, init, 0.0);
  r = fitExpQuadratic({}, {});
  CHECK(!r.converged);
  CHECK_NEAR(r.final_cost, 0.0, 0.0);
  r = fitExpQuadratic({0.0, 0.5, 1.0}, {1.0, 2.0}, init);  // size mismatch
  CHECK(!r.converged);
  CHECK_MAT_NEAR(r.params, init, 0.0);

  // max_iters respected.
  makeData(1.0, 2.0, 1.0, 100, 1.0, 42, x, y);
  r = fitExpQuadratic(x, y, Eigen::Vector3d::Zero(), 2);
  CHECK(r.iterations <= 2);
  CHECK(!r.converged);
#endif
  return TEST_SUMMARY();
}
