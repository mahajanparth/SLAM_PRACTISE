#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

// Markley et al. 2007: the chordal L2 mean of quaternions maximizes q^T M q with
// M = sum_i w_i q_i q_i^T, so it's the eigenvector of M's largest eigenvalue.
// q q^T is invariant to the sign of q, so the double cover is handled automatically.
Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& qs,
                                      const std::vector<double>& weights) {
  if (qs.empty()) return Eigen::Quaterniond::Identity();
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (std::size_t i = 0; i < qs.size(); ++i) {
    const Eigen::Vector4d q = qs[i].coeffs();   // (x, y, z, w); any fixed order works
    M += weights[i] * q * q.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);  // eigenvalues ascending
  Eigen::Vector4d best = es.eigenvectors().col(3);
  Eigen::Quaterniond out;
  out.coeffs() = best.normalized();
  if (out.w() < 0) out.coeffs() *= -1.0;  // canonical sign (optional)
  return out;
}
