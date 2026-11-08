#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

namespace {
std::vector<int> planeInliers(const Cloud& pts, const Eigen::Vector3d& n, double d, double thresh) {
  std::vector<int> in;
  for (int i = 0; i < static_cast<int>(pts.size()); ++i)
    if (std::abs(n.dot(pts[i]) + d) <= thresh) in.push_back(i);
  return in;
}
}  // namespace

bool ransacPlane(const Cloud& pts, double thresh, int iters,
                 Eigen::Vector4d& plane, std::vector<int>& inliers, unsigned seed = 42) {
  const int n_pts = static_cast<int>(pts.size());
  if (n_pts < 3) return false;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> pick(0, n_pts - 1);

  Eigen::Vector3d best_n = Eigen::Vector3d::Zero();
  double best_d = 0.0;
  std::vector<int> best_in;
  bool found = false;

  for (int it = 0; it < iters; ++it) {
    const int a = pick(rng);
    int b = pick(rng), c = pick(rng);
    if (a == b || a == c || b == c) continue;  // need 3 distinct points
    const Eigen::Vector3d e1 = pts[b] - pts[a], e2 = pts[c] - pts[a];
    Eigen::Vector3d n = e1.cross(e2);
    const double nn = n.norm();
    // Relative test: |e1 x e2| = |e1||e2| sin(theta); reject near-collinear/duplicate samples.
    if (nn <= 1e-9 * e1.norm() * e2.norm() || nn == 0.0) continue;
    n /= nn;
    const double d = -n.dot(pts[a]);
    std::vector<int> in = planeInliers(pts, n, d, thresh);
    if (!found || in.size() > best_in.size()) {
      found = true;
      best_n = n;
      best_d = d;
      best_in = std::move(in);
    }
  }
  if (!found) return false;

  // Least-squares refit on the consensus set: centroid + eigenvector of the smallest eigenvalue.
  if (best_in.size() >= 3) {
    Eigen::Vector3d mu = Eigen::Vector3d::Zero();
    for (int i : best_in) mu += pts[i];
    mu /= static_cast<double>(best_in.size());
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (int i : best_in) C += (pts[i] - mu) * (pts[i] - mu).transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);  // eigenvalues ascending
    const Eigen::Vector3d n = es.eigenvectors().col(0);
    const double d = -n.dot(mu);
    std::vector<int> in = planeInliers(pts, n, d, thresh);
    if (in.size() >= best_in.size()) { best_n = n; best_d = d; best_in = std::move(in); }
  }
  plane << best_n, best_d;
  inliers = std::move(best_in);
  return true;
}
