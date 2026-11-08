// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static double angle(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q) {
  // robust angle between rotations (acos near 1 loses ~1e-8 of precision)
  const Eigen::Quaterniond d = p.conjugate() * q;
  return 2.0 * std::atan2(d.vec().norm(), std::abs(d.w()));
}
static Eigen::Quaterniond qaa(double ang, Eigen::Vector3d ax) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(ang, ax.normalized()));
}

int main() {
  Eigen::Quaterniond a = qaa(0.3, {1, 0, 0}), b = qaa(1.5, {0, 1, 1});
  for (double t : {0.0, 0.25, 0.5, 0.9, 1.0}) {
    Eigen::Quaterniond s = slerp(a, b, t);
    CHECK(angle(s, a.slerp(t, b)) < 1e-9);
  }
  Eigen::Quaterniond m = slerp(a, b, 0.5);
  CHECK_NEAR(m.norm(), 1.0, 1e-12);
  CHECK_NEAR(angle(a, m), 0.5 * angle(a, b), 1e-9);
  // random pairs
  bool ok = true;
  for (int i = 0; i < 30; ++i) {
    Eigen::Quaterniond p(cu::randomPose(M_PI, 0).linear()), q(cu::randomPose(M_PI, 0).linear());
    ok &= angle(slerp(p, q, 0.3), p.slerp(0.3, q)) < 1e-9;
  }
  CHECK(ok);

#ifdef HIDDEN
  // shortest path: b and -b give the same result
  Eigen::Quaterniond nb(-b.w(), -b.x(), -b.y(), -b.z());
  CHECK(angle(slerp(a, nb, 0.4), slerp(a, b, 0.4)) < 1e-9);
  CHECK(angle(slerp(a, nb, 0.4), a.slerp(0.4, b)) < 1e-9);
  // a == b and a == -b
  Eigen::Quaterniond s1 = slerp(a, a, 0.7);
  CHECK(s1.coeffs().allFinite());
  CHECK(angle(s1, a) < 1e-9);
  Eigen::Quaterniond na(-a.w(), -a.x(), -a.y(), -a.z());
  Eigen::Quaterniond s2 = slerp(a, na, 0.7);
  CHECK(s2.coeffs().allFinite());
  CHECK(angle(s2, a) < 1e-9);
  // nearly equal (1e-12 rad apart)
  Eigen::Quaterniond c = a * qaa(1e-12, {0, 0, 1});
  Eigen::Quaterniond s3 = slerp(a, c, 0.5);
  CHECK(s3.coeffs().allFinite());
  CHECK_NEAR(s3.norm(), 1.0, 1e-12);
  // 179 deg apart: constant angular velocity (nlerp would fail this)
  Eigen::Quaterniond far = a * qaa(179.0 * M_PI / 180.0, {0.2, -1, 0.4});
  for (double t : {0.1, 0.33, 0.75}) {
    Eigen::Quaterniond s = slerp(a, far, t);
    CHECK_NEAR(angle(a, s), t * angle(a, far), 1e-9);
  }
  // endpoints
  CHECK(angle(slerp(a, far, 0.0), a) < 1e-9);
  CHECK(angle(slerp(a, far, 1.0), far) < 1e-9);
#endif
  return TEST_SUMMARY();
}
