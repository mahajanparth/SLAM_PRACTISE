#include <Eigen/Dense>

struct Reduced {
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
};

Reduced marginalize(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, int na) {
  // TODO: H' = A - B C^-1 B^T, b' = b_a - B C^-1 b_c
  return {};
}

Eigen::VectorXd solveViaSchur(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, int na) {
  // TODO
  return Eigen::VectorXd();
}
