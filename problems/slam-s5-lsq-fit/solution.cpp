#include <Eigen/Dense>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

namespace {
// Centroid + eigen-decomposition of the centred scatter matrix (eigenvalues ascending).
void centredEigen(const Cloud& pts, Eigen::Vector3d& mu, Eigen::Vector3d& evals, Eigen::Matrix3d& evecs) {
  mu.setZero();
  for (const auto& p : pts) mu += p;
  mu /= static_cast<double>(pts.size());
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d q = p - mu;  // centre BEFORE the outer product (no cancellation)
    C += q * q.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  evals = es.eigenvalues();
  evecs = es.eigenvectors();
}
}  // namespace

bool fitPlane(const Cloud& pts, Eigen::Vector3d& normal, double& d) {
  if (pts.size() < 3) return false;
  Eigen::Vector3d mu, ev;
  Eigen::Matrix3d V;
  centredEigen(pts, mu, ev, V);
  // Collinear (or identical): two smallest eigenvalues ~0, so the normal is not unique.
  if (ev(2) <= 0.0 || ev(1) <= 1e-10 * ev(2)) return false;
  normal = V.col(0);  // direction of least variance
  d = -normal.dot(mu);
  return true;
}

bool fitLine(const Cloud& pts, Eigen::Vector3d& point, Eigen::Vector3d& dir) {
  if (pts.size() < 2) return false;
  Eigen::Vector3d mu, ev;
  Eigen::Matrix3d V;
  centredEigen(pts, mu, ev, V);
  if (ev(2) <= 1e-12 * mu.squaredNorm() || ev(2) == 0.0) return false;  // all points identical
  point = mu;
  dir = V.col(2);  // direction of greatest variance
  return true;
}
