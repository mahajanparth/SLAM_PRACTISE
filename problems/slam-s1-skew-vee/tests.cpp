// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  const Eigen::Vector3d a(1, 2, 3), b(-4, 0.5, 2);
  CHECK_MAT_NEAR((skew(a) * b).eval(), a.cross(b), 1e-12);
  CHECK_MAT_NEAR((skew(b) * a).eval(), b.cross(a), 1e-12);
  CHECK_NEAR(skew(a)(0, 1), -3.0, 1e-12);
  CHECK_NEAR(skew(a)(0, 2), 2.0, 1e-12);
  CHECK_MAT_NEAR(vee(skew(a)), a, 1e-12);
  CHECK_MAT_NEAR((skew(a) * a).eval(), Eigen::Vector3d::Zero(), 1e-12);

#ifdef HIDDEN
  CHECK_MAT_NEAR(skew(Eigen::Vector3d::Zero()), Eigen::Matrix3d::Zero(), 0.0);
  CHECK_MAT_NEAR(vee(Eigen::Matrix3d::Zero()), Eigen::Vector3d::Zero(), 0.0);
  const Eigen::Vector3d c(-0.3, 7.5, -1e-9);
  const Eigen::Matrix3d Sc = skew(c);
  CHECK_MAT_NEAR(Sc.transpose(), -Sc, 1e-15);
  CHECK_MAT_NEAR(vee(skew(c)), c, 1e-15);
  // [v]x^2 = v v^T - |v|^2 I
  const Eigen::Matrix3d Sa = skew(a);
  const Eigen::Matrix3d Sa2 = a * a.transpose() - a.squaredNorm() * Eigen::Matrix3d::Identity();
  CHECK_MAT_NEAR((Sa * Sa).eval(), Sa2, 1e-12);
  // vee of a non-exactly-skew matrix -> vee of its skew part
  Eigen::Matrix3d N = skew(a) + Eigen::Matrix3d::Identity() * 0.1;
  N(0, 1) += 0.2; N(1, 0) += 0.2;  // symmetric perturbation must not change vee
  CHECK_MAT_NEAR(vee(N), a, 1e-12);
  // many random vectors
  bool all_ok = true;
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d x = Eigen::Vector3d::Random() * 10, y = Eigen::Vector3d::Random();
    all_ok &= (skew(x) * y - x.cross(y)).norm() < 1e-12;
  }
  CHECK(all_ok);
#endif
  return TEST_SUMMARY();
}
