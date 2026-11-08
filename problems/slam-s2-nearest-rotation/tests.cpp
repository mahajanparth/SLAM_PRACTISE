// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static bool isRotation(const Eigen::Matrix3d& R, double tol) {
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < tol && std::abs(R.determinant() - 1.0) < tol;
}

int main() {
  Eigen::Matrix3d R0 = Eigen::AngleAxisd(0.8, Eigen::Vector3d(1, -2, 0.5).normalized()).toRotationMatrix();
  CHECK_MAT_NEAR(nearestRotation(R0), R0, 1e-12);

  // drifted rotation: small symmetric + antisymmetric noise
  Eigen::Matrix3d N;
  N << 1e-3, -2e-3, 5e-4, 1e-3, -1e-3, 2e-3, -3e-4, 7e-4, 1.5e-3;
  Eigen::Matrix3d M = R0 + N;
  Eigen::Matrix3d R = nearestRotation(M);
  CHECK(isRotation(R, 1e-12));
  CHECK((R - R0).norm() < 1e-2);
  // compare with the closed form via Eigen's polar decomposition: R = M (M^T M)^{-1/2}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(M.transpose() * M);
  Eigen::Matrix3d Rref = M * es.operatorInverseSqrt();
  CHECK_MAT_NEAR(R, Rref, 1e-10);

#ifdef HIDDEN
  // scaled rotation
  CHECK_MAT_NEAR(nearestRotation((2.0 * R0).eval()), R0, 1e-12);
  // det(M) < 0: must still return a proper rotation, the nearest one
  Eigen::Matrix3d D = Eigen::Vector3d(3, 2, -1).asDiagonal();
  Eigen::Matrix3d RD = nearestRotation(D);
  CHECK(isRotation(RD, 1e-12));
  CHECK_MAT_NEAR(RD, Eigen::Matrix3d::Identity(), 1e-12);
  Eigen::Matrix3d Dr = R0 * D;  // rotated version: answer is R0
  CHECK_MAT_NEAR(nearestRotation(Dr), R0, 1e-10);
  // optimality vs random perturbations (Gram-Schmidt fails this: it trusts the first column)
  Eigen::Matrix3d B;
  B << 1.0, 0.3, -0.2, 0.1, 0.9, 0.25, -0.15, 0.2, 1.1;
  Eigen::Matrix3d RB = nearestRotation(B);
  CHECK(isRotation(RB, 1e-12));
  bool optimal = true;
  for (int i = 0; i < 200; ++i) {
    Eigen::Matrix3d P = Eigen::AngleAxisd(0.05, Eigen::Vector3d::Random().normalized()).toRotationMatrix() * RB;
    optimal &= (RB - B).norm() <= (P - B).norm() + 1e-12;
  }
  CHECK(optimal);
  // random drifted rotations
  bool ok = true;
  for (int i = 0; i < 30; ++i) {
    Eigen::Matrix3d Rr = cu::randomPose(M_PI, 0).linear();
    Eigen::Matrix3d Mr = Rr + 1e-4 * Eigen::Matrix3d::Random();
    Eigen::Matrix3d Rn = nearestRotation(Mr);
    ok &= isRotation(Rn, 1e-12) && (Rn - Rr).norm() < 1e-3;
  }
  CHECK(ok);
#endif
  return TEST_SUMMARY();
}
