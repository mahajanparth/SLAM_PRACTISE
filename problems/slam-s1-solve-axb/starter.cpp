#include <Eigen/Dense>

Eigen::VectorXd solveLLT(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  // TODO (return an empty vector if A is not SPD)
  return Eigen::VectorXd::Zero(b.size());
}

Eigen::VectorXd solveQR(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  // TODO
  return Eigen::VectorXd::Zero(b.size());
}

Eigen::VectorXd solveSVD(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  // TODO
  return Eigen::VectorXd::Zero(A.cols());
}

Eigen::VectorXd leastSquares(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  // TODO
  return Eigen::VectorXd::Zero(A.cols());
}

double residual(const Eigen::MatrixXd& A, const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
  // TODO
  return -1.0;
}
