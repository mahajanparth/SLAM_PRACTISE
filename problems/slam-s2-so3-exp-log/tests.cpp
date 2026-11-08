// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static Eigen::Matrix3d aa(double angle, const Eigen::Vector3d& axis) {
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

int main() {
  const Eigen::Vector3d w(0.3, -0.2, 0.5);
  CHECK_MAT_NEAR(expSO3(w), aa(w.norm(), w), 1e-12);
  CHECK_MAT_NEAR(expSO3(Eigen::Vector3d(0, 0, M_PI / 2)), aa(M_PI / 2, Eigen::Vector3d::UnitZ()), 1e-12);
  CHECK_MAT_NEAR(logSO3(aa(1.2, Eigen::Vector3d(1, 1, 0))), (1.2 * Eigen::Vector3d(1, 1, 0).normalized()).eval(), 1e-12);
  CHECK_MAT_NEAR(logSO3(expSO3(w)), w, 1e-12);
  CHECK_MAT_NEAR(expSO3(Eigen::Vector3d::Zero()), Eigen::Matrix3d::Identity(), 1e-15);

  bool rt = true;
  for (int i = 0; i < 50; ++i) {
    Eigen::Vector3d r = Eigen::Vector3d::Random() * 1.7;  // |r| < pi
    rt &= (logSO3(expSO3(r)) - r).norm() < 1e-10;
    rt &= (expSO3(r) - aa(r.norm(), r)).norm() < 1e-12;
  }
  CHECK(rt);

#ifdef HIDDEN
  // exact zero and tiny angles: no NaN, still accurate
  CHECK_MAT_NEAR(logSO3(Eigen::Matrix3d::Identity()), Eigen::Vector3d::Zero(), 1e-15);
  const Eigen::Vector3d tiny(1e-9, -2e-9, 0.5e-9);
  Eigen::Matrix3d Rt = expSO3(tiny);
  CHECK(Rt.allFinite());
  CHECK_MAT_NEAR(Rt, (Eigen::Matrix3d::Identity() + skew(tiny)).eval(), 1e-15);
  CHECK_MAT_NEAR(logSO3(aa(tiny.norm(), tiny)), tiny, 1e-15);
  // trace slightly > 3 from rounding: acos argument must be clamped
  Eigen::Matrix3d Rup = Eigen::Matrix3d::Identity() * (1.0 + 1e-14);
  CHECK(logSO3(Rup).allFinite());
  CHECK(logSO3(Rup).norm() < 1e-12);
  // exactly pi about several axes: exp(log(R)) == R and |log| == pi
  const Eigen::Vector3d axes[] = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d(1, 2, -2),
                                  Eigen::Vector3d(-0.3, 0.1, 1)};
  for (const auto& ax : axes) {
    Eigen::Matrix3d Rpi = aa(M_PI, ax);
    Eigen::Vector3d l = logSO3(Rpi);
    CHECK(l.allFinite());
    CHECK_NEAR(l.norm(), M_PI, 1e-9);
    CHECK_MAT_NEAR(expSO3(l), Rpi, 1e-9);
  }
  // just below pi: the sign of the axis is determined
  const Eigen::Vector3d ax(0.6, -0.8, 0.0);
  for (double eps : {1e-4, 1e-7, 1e-10}) {
    Eigen::Vector3d l = logSO3(aa(M_PI - eps, ax));
    CHECK_MAT_NEAR(l, ((M_PI - eps) * ax).eval(), 1e-6);
  }
  // non-trivial rotation of angle > pi wraps to |w| <= pi
  Eigen::Vector3d lw = logSO3(aa(1.5 * M_PI, Eigen::Vector3d::UnitZ()));
  CHECK_MAT_NEAR(lw, Eigen::Vector3d(0, 0, -0.5 * M_PI), 1e-12);
#endif
  return TEST_SUMMARY();
}
