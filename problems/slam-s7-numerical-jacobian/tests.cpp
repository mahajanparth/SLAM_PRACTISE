// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  // f: R^3 -> R^2, analytic Jacobian known.
  auto f = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::VectorXd y(2);
    y << x[0] * x[1] + std::sin(x[2]), x[0] * x[0] * x[2];
    return y;
  };
  Eigen::VectorXd x0(3);
  x0 << 0.5, -1.2, 0.3;
  Eigen::MatrixXd Jtrue(2, 3);
  Jtrue << x0[1], x0[0], std::cos(x0[2]),
           2 * x0[0] * x0[2], 0, x0[0] * x0[0];
  Eigen::MatrixXd J = numericalJacobian(f, x0);
  CHECK(J.rows() == 2 && J.cols() == 3);
  CHECK_MAT_NEAR(J, Jtrue, 1e-8);

  // (wrap Eigen expressions in concrete types: CHECK_MAT_NEAR binds them by const&)
  CHECK_MAT_NEAR(Eigen::Vector3d(skew(Eigen::Vector3d(1, 2, 3)) * Eigen::Vector3d(-4, 5, 0.5)),
                 Eigen::Vector3d(Eigen::Vector3d(1, 2, 3).cross(Eigen::Vector3d(-4, 5, 0.5))), 1e-12);

  const Eigen::Matrix3d R = expSO3(Eigen::Vector3d(0.3, -0.5, 0.8));
  const Eigen::Vector3d p(1.0, 2.0, -0.5);
  const Eigen::Matrix3d Ja = rotatePointJacobianAnalytic(R, p);
  const Eigen::Matrix3d Jn = rotatePointJacobianNumerical(R, p);
  CHECK(Ja.norm() > 1e-3);
  CHECK_MAT_NEAR(Ja, Jn, 1e-8);
  CHECK_MAT_NEAR(Ja, Eigen::Matrix3d(-R * Eigen::Matrix3d((Eigen::Matrix3d() << 0, 0.5, 2, -0.5, 0, -1, -2, 1, 0).finished())), 1e-12);

#ifdef HIDDEN
  // Central (not forward) differences: cubic has 3rd-derivative error only ~eps^2.
  auto cubic = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::VectorXd y(1);
    y << x[0] * x[0] * x[0];
    return y;
  };
  Eigen::VectorXd one = Eigen::VectorXd::Constant(1, 2.0);
  Eigen::MatrixXd Jc = numericalJacobian(cubic, one, 1e-3);
  CHECK(Jc.rows() == 1 && Jc.cols() == 1);
  if (Jc.size() == 1) CHECK_NEAR(Jc(0, 0), 12.0, 1.5e-6);  // forward diff would be off by ~6e-3

  // Wide output (m > n) and evaluation count.
  int calls = 0;
  auto g = [&calls](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    ++calls;
    Eigen::VectorXd y(4);
    y << x[0], 2 * x[1], x[0] * x[1], std::exp(x[0]);
    return y;
  };
  Eigen::VectorXd x2(2);
  x2 << 0.1, -0.7;
  Eigen::MatrixXd Jg = numericalJacobian(g, x2);
  Eigen::MatrixXd Jg_true(4, 2);
  Jg_true << 1, 0, 0, 2, x2[1], x2[0], std::exp(x2[0]), 0;
  CHECK_MAT_NEAR(Jg, Jg_true, 1e-8);
  CHECK(calls <= 2 * 2 + 1);

  // Skew-symmetry.
  const Eigen::Matrix3d S = skew(Eigen::Vector3d(-0.3, 4, 1e3));
  CHECK_MAT_NEAR(S, Eigen::Matrix3d(-S.transpose()), 0.0);

  // Random rotations including near-pi, plus p = 0.
  for (int t = 0; t < 10; ++t) {
    const Eigen::Matrix3d Rr = cu::randomPose(t < 3 ? M_PI - 1e-3 : M_PI).linear();
    const Eigen::Vector3d pr = cu::uniformCube(1, 5.0)[0];
    CHECK_MAT_NEAR(rotatePointJacobianAnalytic(Rr, pr), rotatePointJacobianNumerical(Rr, pr), 1e-7);
  }
  CHECK_MAT_NEAR(rotatePointJacobianAnalytic(R, Eigen::Vector3d::Zero()), Eigen::Matrix3d::Zero(), 0.0);
  // Identity rotation: J = -[p]x.
  CHECK_MAT_NEAR(rotatePointJacobianNumerical(Eigen::Matrix3d::Identity(), p), Eigen::Matrix3d(-skew(p)), 1e-8);
#endif
  return TEST_SUMMARY();
}
