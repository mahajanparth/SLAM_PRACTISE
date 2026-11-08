#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

double huberWeight(double r, double k) {
  const double a = std::abs(r);
  return a <= k ? 1.0 : k / a;  // rho'(r)/r of the Huber loss
}

namespace {
double median(std::vector<double>& v) {  // reorders v; v must be non-empty
  const std::size_t n = v.size(), mid = n / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  const double hi = v[mid];
  if (n % 2) return hi;
  const double lo = *std::max_element(v.begin(), v.begin() + mid);  // largest of the lower half
  return 0.5 * (lo + hi);
}

// Weighted LS for y = m x + c. Returns false if the 2x2 system is (near) singular.
bool solveWeighted(const std::vector<Eigen::Vector2d>& pts, const std::vector<double>& w,
                   Eigen::Vector2d& theta) {
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  Eigen::Vector2d rhs = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const Eigen::Vector2d j(pts[i].x(), 1.0);
    A.noalias() += w[i] * j * j.transpose();
    rhs.noalias() += w[i] * j * pts[i].y();
  }
  // Scale-aware singularity test: det relative to the diagonal product.
  const double det = A.determinant();
  if (!(std::abs(det) > 1e-12 * std::max(1.0, A(0, 0) * A(1, 1)))) return false;
  theta = A.ldlt().solve(rhs);
  return true;
}
}  // namespace

double madSigma(std::vector<double> r) {
  if (r.empty()) return 0.0;
  const double med = median(r);
  for (double& v : r) v = std::abs(v - med);
  return 1.4826 * median(r);  // 1.4826 makes MAD consistent with sigma for Gaussian noise
}

Eigen::Vector2d fitLineLS(const std::vector<Eigen::Vector2d>& pts) {
  Eigen::Vector2d theta = Eigen::Vector2d::Zero();
  if (pts.size() < 2 || !solveWeighted(pts, std::vector<double>(pts.size(), 1.0), theta))
    return Eigen::Vector2d::Zero();
  return theta;
}

Eigen::Vector2d fitLineHuber(const std::vector<Eigen::Vector2d>& pts, double k, int iters = 20,
                             std::vector<double>* weights = nullptr) {
  std::vector<double> w(pts.size(), 1.0);
  Eigen::Vector2d theta = Eigen::Vector2d::Zero();
  if (pts.size() < 2 || !solveWeighted(pts, w, theta)) {
    if (weights) *weights = w;
    return Eigen::Vector2d::Zero();
  }
  for (int it = 0; it < iters; ++it) {
    for (std::size_t i = 0; i < pts.size(); ++i)
      w[i] = huberWeight(pts[i].y() - (theta[0] * pts[i].x() + theta[1]), k);
    Eigen::Vector2d next;
    if (!solveWeighted(pts, w, next)) break;
    const bool done = (next - theta).norm() < 1e-12;
    theta = next;
    if (done) break;
  }
  if (weights) {  // weights consistent with the returned parameters
    for (std::size_t i = 0; i < pts.size(); ++i)
      w[i] = huberWeight(pts[i].y() - (theta[0] * pts[i].x() + theta[1]), k);
    *weights = w;
  }
  return theta;
}
