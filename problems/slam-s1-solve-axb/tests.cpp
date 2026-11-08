// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static Eigen::MatrixXd randMat(int r, int c) {
  std::uniform_real_distribution<double> u(-1, 1);
  Eigen::MatrixXd M(r, c);
  for (int j = 0; j < c; ++j)
    for (int i = 0; i < r; ++i) M(i, j) = u(cu::rng());
  return M;
}

// m x n matrix with prescribed condition number: U * diag(s) * V^T.
static Eigen::MatrixXd withCond(int m, int n, double cond) {
  Eigen::MatrixXd U = randMat(m, n).householderQr().householderQ() * Eigen::MatrixXd::Identity(m, n);
  Eigen::MatrixXd V = randMat(n, n).householderQr().householderQ();
  Eigen::VectorXd s(n);
  for (int i = 0; i < n; ++i) s(i) = std::pow(cond, -double(i) / (n - 1));
  return U * s.asDiagonal() * V.transpose();
}

int main() {
  const int n = 6;
  Eigen::MatrixXd B = randMat(n, n);
  Eigen::MatrixXd A = B * B.transpose() + n * Eigen::MatrixXd::Identity(n, n);  // SPD
  Eigen::VectorXd xt = randMat(n, 1);
  Eigen::VectorXd b = A * xt;

  Eigen::VectorXd x1 = solveLLT(A, b), x2 = solveQR(A, b), x3 = solveSVD(A, b);
  CHECK(x1.size() == n && x2.size() == n && x3.size() == n);
  if (x1.size() == n) CHECK_MAT_NEAR(x1, xt, 1e-9);
  if (x2.size() == n) CHECK_MAT_NEAR(x2, xt, 1e-9);
  if (x3.size() == n) CHECK_MAT_NEAR(x3, xt, 1e-9);
  CHECK_NEAR(residual(A, xt, b), 0.0, 1e-10);
  CHECK_NEAR(residual(Eigen::MatrixXd::Identity(2, 2), Eigen::Vector2d(1, 1), Eigen::Vector2d(4, 5)), 5.0, 1e-12);

  // overdetermined least squares: compare with the normal equations on a well-conditioned problem
  Eigen::MatrixXd T = randMat(30, 4);
  Eigen::VectorXd y = randMat(30, 1);
  Eigen::VectorXd xls = leastSquares(T, y);
  Eigen::VectorXd xne = (T.transpose() * T).ldlt().solve(T.transpose() * y);
  CHECK(xls.size() == 4);
  if (xls.size() == 4) CHECK_MAT_NEAR(xls, xne, 1e-9);

#ifdef HIDDEN
  // Not SPD (indefinite) -> LLT must report failure with an empty vector.
  Eigen::MatrixXd Ind = Eigen::Vector3d(1, -2, 3).asDiagonal();
  CHECK(solveLLT(Ind, Eigen::Vector3d(1, 1, 1)).size() == 0);
  // ...but QR and SVD handle it.
  CHECK_MAT_NEAR(solveQR(Ind, Eigen::Vector3d(1, 1, 1)), Eigen::Vector3d(1, -0.5, 1.0 / 3), 1e-12);
  CHECK_MAT_NEAR(solveSVD(Ind, Eigen::Vector3d(1, 1, 1)), Eigen::Vector3d(1, -0.5, 1.0 / 3), 1e-12);

  // Rank-deficient: SVD gives the minimum-norm solution.
  Eigen::MatrixXd R2(2, 2); R2 << 1, 1, 1, 1;
  CHECK_MAT_NEAR(solveSVD(R2, Eigen::Vector2d(2, 2)), Eigen::Vector2d(1, 1), 1e-12);

  // 1x1 system
  Eigen::MatrixXd one(1, 1); one << 4;
  Eigen::VectorXd bo(1); bo << 2;
  CHECK_MAT_NEAR(solveLLT(one, bo), Eigen::VectorXd::Constant(1, 0.5), 1e-12);

  // Ill-conditioned tall least squares (cond 1e7): normal equations lose ~cond^2*eps ~ 1e-2.
  Eigen::MatrixXd Ac = withCond(60, 6, 1e7);
  Eigen::VectorXd xc = randMat(6, 1);
  Eigen::VectorXd bc = Ac * xc;  // consistent -> exact solution is xc
  Eigen::VectorXd xcls = leastSquares(Ac, bc);
  CHECK(xcls.size() == 6);
  if (xcls.size() == 6) CHECK((xcls - xc).norm() < 1e-5);

  // Inconsistent tall system: the LS residual must be orthogonal to range(A).
  Eigen::MatrixXd At = randMat(50, 3);
  Eigen::VectorXd bt = randMat(50, 1);
  Eigen::VectorXd xt2 = leastSquares(At, bt);
  if (xt2.size() == 3) CHECK((At.transpose() * (At * xt2 - bt)).norm() < 1e-10);
  else CHECK(false);
  CHECK(residual(At, xt2, bt) <= residual(At, xt2 + Eigen::Vector3d(1e-3, 0, 0), bt));
#endif
  return TEST_SUMMARY();
}
