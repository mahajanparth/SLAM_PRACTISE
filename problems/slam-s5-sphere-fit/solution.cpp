#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

// |p - c|^2 = r^2  <=>  |p|^2 = 2 c.p + k,  k = r^2 - |c|^2  -> linear in (c, k).
bool fitSphere(const Cloud& pts, Eigen::Vector3d& center, double& radius) {
  const int n = static_cast<int>(pts.size());
  if (n < 4) return false;
  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  for (const auto& p : pts) mu += p;
  mu /= n;  // centre for conditioning (|p|^2 ~ 1e6 at map scale otherwise)

  Eigen::MatrixX4d A(n, 4);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d q = pts[i] - mu;
    A.row(i) << 2.0 * q.transpose(), 1.0;
    b(i) = q.squaredNorm();
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixX4d> qr(A);
  qr.setThreshold(1e-10);
  if (qr.rank() < 4) return false;  // coplanar / too few distinct points
  const Eigen::Vector4d x = qr.solve(b);
  const double r2 = x(3) + x.head<3>().squaredNorm();
  if (!(r2 > 0.0)) return false;
  center = mu + x.head<3>();
  radius = std::sqrt(r2);
  return true;
}

namespace {
std::vector<int> sphereInliers(const Cloud& pts, const Eigen::Vector3d& c, double r, double thresh) {
  std::vector<int> in;
  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
    if (std::abs((pts[i] - c).norm() - r) <= thresh) in.push_back(i);
  return in;
}
}  // namespace

bool ransacSphere(const Cloud& pts, double thresh, int iters,
                  Eigen::Vector3d& center, double& radius, std::vector<int>& inliers,
                  unsigned seed = 42) {
  const int n = static_cast<int>(pts.size());
  if (n < 4) return false;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> pick(0, n - 1);
  bool found = false;
  Eigen::Vector3d best_c = Eigen::Vector3d::Zero();
  double best_r = 0.0;
  std::vector<int> best_in;

  for (int it = 0; it < iters; ++it) {
    int s[4];
    for (int& k : s) k = pick(rng);
    if (s[0] == s[1] || s[0] == s[2] || s[0] == s[3] || s[1] == s[2] || s[1] == s[3] || s[2] == s[3]) continue;
    Eigen::Vector3d c;
    double r;
    if (!fitSphere({pts[s[0]], pts[s[1]], pts[s[2]], pts[s[3]]}, c, r)) continue;  // degenerate sample
    std::vector<int> in = sphereInliers(pts, c, r, thresh);
    if (!found || in.size() > best_in.size()) {
      found = true;
      best_c = c;
      best_r = r;
      best_in = std::move(in);
    }
  }
  if (!found) return false;

  // Refit on the consensus set, keep it if it doesn't lose support.
  Cloud sub;
  sub.reserve(best_in.size());
  for (int i : best_in) sub.push_back(pts[i]);
  Eigen::Vector3d c;
  double r;
  if (fitSphere(sub, c, r)) {
    std::vector<int> in = sphereInliers(pts, c, r, thresh);
    if (in.size() >= best_in.size()) { best_c = c; best_r = r; best_in = std::move(in); }
  }
  center = best_c;
  radius = best_r;
  inliers = std::move(best_in);
  return true;
}
