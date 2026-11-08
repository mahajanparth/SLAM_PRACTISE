#include <Eigen/Dense>
#include <Eigen/SVD>

// Orthogonal Procrustes: maximize tr(R^T M) over SO(3).
// With M = U S V^T the optimum is U D V^T with D = diag(1, 1, det(U V^T)): if U V^T is a reflection,
// flip the direction paired with the SMALLEST singular value (that costs the least).
Eigen::Matrix3d nearestRotation(const Eigen::Matrix3d& M) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d& U = svd.matrixU();
  const Eigen::Matrix3d& V = svd.matrixV();
  Eigen::Vector3d d(1.0, 1.0, (U * V.transpose()).determinant() < 0 ? -1.0 : 1.0);
  return U * d.asDiagonal() * V.transpose();
}
