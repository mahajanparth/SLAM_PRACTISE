// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static Eigen::Matrix3d ref(double y, double p, double r) {
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
static double angDiff(double a, double b) { return std::abs(std::remainder(a - b, 2 * M_PI)); }

int main() {
  CHECK_MAT_NEAR(eulerZYXToRot(0.3, -0.4, 1.1), ref(0.3, -0.4, 1.1), 1e-12);
  CHECK_MAT_NEAR(eulerZYXToRot(2.5, 1.2, -3.0), ref(2.5, 1.2, -3.0), 1e-12);
  Eigen::Vector3d e = rotToEulerZYX(ref(0.3, -0.4, 1.1));
  CHECK_NEAR(e(0), 0.3, 1e-12);
  CHECK_NEAR(e(1), -0.4, 1e-12);
  CHECK_NEAR(e(2), 1.1, 1e-12);
  // round trip on random rotations
  bool rt = true;
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix3d R = cu::randomPose(M_PI, 0).linear();
    Eigen::Vector3d a = rotToEulerZYX(R);
    rt &= (eulerZYXToRot(a(0), a(1), a(2)) - R).norm() < 1e-9;
    rt &= std::abs(a(1)) <= M_PI / 2 + 1e-12;
  }
  CHECK(rt);

#ifdef HIDDEN
  // Gimbal lock: pitch exactly +-90 deg -> roll = 0, matrix round trip still exact
  for (double p : {M_PI / 2, -M_PI / 2}) {
    Eigen::Matrix3d R = ref(0.7, p, 0.25);
    Eigen::Vector3d a = rotToEulerZYX(R);
    CHECK(a.allFinite());
    CHECK_NEAR(a(1), p, 1e-9);
    CHECK_NEAR(a(2), 0.0, 1e-12);
    CHECK_MAT_NEAR(eulerZYXToRot(a(0), a(1), a(2)), R, 1e-9);
  }
  // pitch = 89.9999 deg: NOT locked, angles must still come back
  const double p = 89.9999 * M_PI / 180.0;
  Eigen::Vector3d a = rotToEulerZYX(ref(-1.0, p, 0.5));
  CHECK_NEAR(a(0), -1.0, 1e-6);
  CHECK_NEAR(a(1), p, 1e-9);
  CHECK_NEAR(a(2), 0.5, 1e-6);
  // yaw wrap-around near +-pi
  Eigen::Vector3d w1 = rotToEulerZYX(ref(M_PI - 1e-9, 0.1, 0.2));
  CHECK(angDiff(w1(0), M_PI - 1e-9) < 1e-9);
  Eigen::Vector3d w2 = rotToEulerZYX(ref(-M_PI + 1e-9, 0.1, 0.2));
  CHECK(angDiff(w2(0), -M_PI + 1e-9) < 1e-9);
  // yaw = 3*pi/2 comes back as -pi/2
  Eigen::Vector3d w3 = rotToEulerZYX(eulerZYXToRot(1.5 * M_PI, 0, 0));
  CHECK_NEAR(w3(0), -0.5 * M_PI, 1e-12);
  // identity
  CHECK_MAT_NEAR(rotToEulerZYX(Eigen::Matrix3d::Identity()), Eigen::Vector3d::Zero(), 1e-15);
#endif
  return TEST_SUMMARY();
}
