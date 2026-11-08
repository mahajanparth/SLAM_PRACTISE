#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// r_i(δ) = n·((I+[ω]x) p + v - q) = n·(p - q) + ω·(p × n) + n·v
//   => r_i(0) = n·(p - q),   J_i = [ (p × n)ᵀ, nᵀ ]
Vector6d pointToPlaneStep(const Cloud& p, const Cloud& q, const Cloud& n) {
  if (p.size() != q.size() || p.size() != n.size() || p.size() < 6) return Vector6d::Zero();
  Matrix6d H = Matrix6d::Zero();
  Vector6d b = Vector6d::Zero();
  for (size_t i = 0; i < p.size(); ++i) {
    Vector6d J;
    J << p[i].cross(n[i]), n[i];
    const double r = n[i].dot(p[i] - q[i]);
    H.noalias() += J * J.transpose();
    b += J * r;
  }
  // Degeneracy (e.g. a single plane, a corridor): some direction of δ is unobservable.
  Eigen::SelfAdjointEigenSolver<Matrix6d> es(H);
  const Vector6d ev = es.eigenvalues();  // ascending
  if (!(ev(5) > 0.0) || ev(0) <= 1e-9 * ev(5)) return Vector6d::Zero();
  return H.ldlt().solve(-b);  // δ = -H⁻¹ b
}

Eigen::Isometry3d applyUpdate(const Vector6d& delta, const Eigen::Isometry3d& T) {
  const Eigen::Vector3d w = delta.head<3>();
  const double angle = w.norm();
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
  if (angle > 1e-15) dT.linear() = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();  // exact Exp, stays on SO(3)
  dT.translation() = delta.tail<3>();
  return dT * T;  // left update: the points were already expressed with T applied
}
