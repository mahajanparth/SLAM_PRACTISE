// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

static Eigen::MatrixXd randomSPD(int n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1, 1);
  Eigen::MatrixXd M(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) M(i, j) = u(rng);
  return M * M.transpose() + n * Eigen::MatrixXd::Identity(n, n);
}
static Eigen::VectorXd randomVec(int n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-5, 5);
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) v[i] = u(rng);
  return v;
}

int main() {
  // 2x2 by hand: H = [[4,2],[2,2]], b = [2,4] -> H' = 4 - 2*2/2 = 2, b' = 2 - 2*4/2 = -2.
  Eigen::MatrixXd H2(2, 2);
  H2 << 4, 2, 2, 2;
  Eigen::VectorXd b2(2);
  b2 << 2, 4;
  Reduced r2 = marginalize(H2, b2, 1);
  CHECK(r2.H.rows() == 1 && r2.H.cols() == 1 && r2.b.size() == 1);
  if (r2.H.size() == 1 && r2.b.size() == 1) {
    CHECK_NEAR(r2.H(0, 0), 2.0, 1e-12);
    CHECK_NEAR(r2.b[0], -2.0, 1e-12);
  }

  const Eigen::MatrixXd H = randomSPD(9, 1);
  const Eigen::VectorXd b = randomVec(9, 2);
  const Eigen::VectorXd x_ref = H.ldlt().solve(b);
  const Eigen::VectorXd x = solveViaSchur(H, b, 6);
  CHECK_MAT_NEAR(x, x_ref, 1e-9);
  // Reduced system's solution equals the first block of the full solution.
  Reduced r = marginalize(H, b, 6);
  CHECK(r.H.rows() == 6 && r.H.cols() == 6 && r.b.size() == 6);
  if (r.H.rows() == 6 && r.b.size() == 6) CHECK_MAT_NEAR(Eigen::VectorXd(r.H.ldlt().solve(r.b)), Eigen::VectorXd(x_ref.head(6)), 1e-9);

#ifdef HIDDEN
  // Covariance consistency: inv(H') == top-left block of inv(H); H' symmetric.
  if (r.H.rows() == 6) {
    const Eigen::MatrixXd cov = H.inverse();
    CHECK_MAT_NEAR(Eigen::MatrixXd(r.H.inverse()), Eigen::MatrixXd(cov.topLeftCorner(6, 6)), 1e-9);
    CHECK_MAT_NEAR(r.H, Eigen::MatrixXd(r.H.transpose()), 1e-10);
  }

  // B = 0 (decoupled): H' = A, b' = b_a.
  Eigen::MatrixXd Hd = Eigen::MatrixXd::Zero(5, 5);
  Hd.topLeftCorner(2, 2) = randomSPD(2, 3);
  Hd.bottomRightCorner(3, 3) = randomSPD(3, 4);
  const Eigen::VectorXd bd = randomVec(5, 5);
  Reduced rd = marginalize(Hd, bd, 2);
  CHECK(rd.H.rows() == 2 && rd.b.size() == 2);
  if (rd.H.rows() == 2 && rd.b.size() == 2) {
    CHECK_MAT_NEAR(rd.H, Eigen::MatrixXd(Hd.topLeftCorner(2, 2)), 1e-12);
    CHECK_MAT_NEAR(rd.b, Eigen::VectorXd(bd.head(2)), 1e-12);
  }

  // na == n: nothing eliminated.
  Reduced rn = marginalize(H, b, 9);
  CHECK(rn.H.rows() == 9 && rn.b.size() == 9);
  if (rn.H.rows() == 9 && rn.b.size() == 9) CHECK_MAT_NEAR(rn.H, H, 0.0);
  CHECK_MAT_NEAR(solveViaSchur(H, b, 9), x_ref, 1e-9);

  // na == 0: empty reduced system, solve still works.
  Reduced r0 = marginalize(H, b, 0);
  CHECK(r0.H.rows() == 0 && r0.H.cols() == 0 && r0.b.size() == 0);
  CHECK_MAT_NEAR(solveViaSchur(H, b, 0), x_ref, 1e-9);

  // BA-like: 2 cameras (6 dof) + 40 landmarks (3 dof), arrow structure, landmarks eliminated.
  const int nc = 12, np = 120, n = nc + np;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1, 1);
  Eigen::MatrixXd Jt = Eigen::MatrixXd::Zero(n, 2 * 2 * 40);  // J^T, each obs touches 1 cam + 1 point
  int col = 0;
  for (int p = 0; p < 40; ++p)
    for (int c = 0; c < 2; ++c)
      for (int k = 0; k < 2; ++k, ++col) {
        for (int d = 0; d < 6; ++d) Jt(6 * c + d, col) = u(rng);
        for (int d = 0; d < 3; ++d) Jt(nc + 3 * p + d, col) = u(rng);
      }
  const Eigen::MatrixXd Hba = Jt * Jt.transpose() + 1e-3 * Eigen::MatrixXd::Identity(n, n);
  const Eigen::VectorXd bba = randomVec(n, 8);
  CHECK_MAT_NEAR(solveViaSchur(Hba, bba, nc), Eigen::VectorXd(Hba.ldlt().solve(bba)), 1e-6);
  Reduced rba = marginalize(Hba, bba, nc);
  CHECK(rba.H.rows() == nc && rba.H.cols() == nc);
#endif
  return TEST_SUMMARY();
}
