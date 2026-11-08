#include <Eigen/Dense>

// Cholesky: ~n^3/3 flops, the fastest, but only valid for SPD matrices. It never throws,
// so you MUST check info().
Eigen::VectorXd solveLLT(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  Eigen::LLT<Eigen::MatrixXd> llt(A);
  if (llt.info() != Eigen::Success) return Eigen::VectorXd();
  return llt.solve(b);
}

// Rank-revealing QR: works for any square (or tall) matrix, ~2x the cost of LU.
Eigen::VectorXd solveQR(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  return A.colPivHouseholderQr().solve(b);
}

// SVD: the most robust and the slowest. Singular values below threshold() are treated as zero,
// which gives the minimum-norm solution for rank-deficient systems.
Eigen::VectorXd solveSVD(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.solve(b);
}

// QR on the tall matrix directly: error ~ cond(A)*eps. The normal equations
// (A^T A).ldlt().solve(A^T b) are faster but have error ~ cond(A)^2 * eps.
Eigen::VectorXd leastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  return A.colPivHouseholderQr().solve(b);
}

double residual(const Eigen::MatrixXd& A, const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
  return (A * x - b).norm();
}
