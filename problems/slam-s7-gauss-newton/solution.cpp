#include <Eigen/Dense>
#include <cmath>
#include <vector>

struct GNResult {
  Eigen::Vector3d params = Eigen::Vector3d::Zero();  // (a, b, c)
  int iterations = 0;
  bool converged = false;
  double final_cost = 0.0;
};

namespace {
double cost(const std::vector<double>& x, const std::vector<double>& y, const Eigen::Vector3d& p) {
  double c = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double r = y[i] - std::exp(p[0] * x[i] * x[i] + p[1] * x[i] + p[2]);
    c += r * r;
  }
  return 0.5 * c;
}
}  // namespace

GNResult fitExpQuadratic(const std::vector<double>& x, const std::vector<double>& y,
                         const Eigen::Vector3d& init = Eigen::Vector3d::Zero(), int max_iters = 50) {
  GNResult res;
  res.params = init;
  if (x.size() != y.size() || x.size() < 3) {
    res.final_cost = x.size() == y.size() ? cost(x, y, init) : 0.0;
    return res;
  }
  Eigen::Vector3d& p = res.params;
  for (int it = 0; it < max_iters; ++it) {
    // Accumulate the 3x3 normal equations directly: H = sum J_i^T J_i, g = sum J_i^T r_i.
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < x.size(); ++i) {
      const double xi = x[i];
      const double f = std::exp(p[0] * xi * xi + p[1] * xi + p[2]);
      const double r = y[i] - f;
      const Eigen::Vector3d J(-f * xi * xi, -f * xi, -f);  // dr/d(a,b,c)
      H.noalias() += J * J.transpose();
      g.noalias() += J * r;
    }
    Eigen::Vector3d dx = H.ldlt().solve(-g);
    if (!dx.allFinite()) break;  // singular / degenerate data
    // Backtracking: the full GN step can overshoot wildly far from the optimum
    // (exp() is very non-linear), so halve it until the cost does not increase.
    const double c0 = cost(x, y, p);
    for (int k = 0; k < 30 && !(cost(x, y, p + dx) <= c0); ++k) dx *= 0.5;
    p += dx;
    ++res.iterations;
    if (dx.norm() < 1e-10) { res.converged = true; break; }
  }
  res.final_cost = cost(x, y, p);
  return res;
}
