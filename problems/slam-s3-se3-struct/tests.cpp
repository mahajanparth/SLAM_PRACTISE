// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static SE3 fromIso(const Eigen::Isometry3d& T) {
  SE3 s;
  s.R = T.linear();
  s.t = T.translation();
  return s;
}

int main() {
  const Eigen::Isometry3d A = cu::randomPose(M_PI, 2.0), B = cu::randomPose(M_PI, 2.0);
  const SE3 a = fromIso(A), b = fromIso(B);
  const Eigen::Vector3d p(0.3, -1.0, 2.5);

  CHECK_MAT_NEAR(a * p, A * p, 1e-12);
  const SE3 ab = a * b;
  CHECK_MAT_NEAR(ab.R, (A * B).linear().eval(), 1e-12);
  CHECK_MAT_NEAR(ab.t, (A * B).translation().eval(), 1e-12);
  const SE3 ai = a.inverse();
  CHECK_MAT_NEAR(ai.R, A.inverse().linear().eval(), 1e-12);
  CHECK_MAT_NEAR(ai.t, A.inverse().translation().eval(), 1e-12);
  CHECK_MAT_NEAR(a.matrix(), A.matrix(), 1e-12);

#ifdef HIDDEN
  // T * T^-1 = I, and both orders
  const SE3 i1 = a * a.inverse(), i2 = a.inverse() * a;
  CHECK_MAT_NEAR(i1.R, Eigen::Matrix3d::Identity(), 1e-12);
  CHECK_MAT_NEAR(i1.t, Eigen::Vector3d::Zero(), 1e-12);
  CHECK_MAT_NEAR(i2.t, Eigen::Vector3d::Zero(), 1e-12);
  // associativity
  const SE3 c = fromIso(cu::randomPose(M_PI, 5.0));
  const SE3 l = (a * b) * c, r = a * (b * c);
  CHECK_MAT_NEAR(l.R, r.R, 1e-12);
  CHECK_MAT_NEAR(l.t, r.t, 1e-12);
  // composition acts in the right order on points
  CHECK_MAT_NEAR((a * b) * p, a * (b * p), 1e-12);
  // (ab)^-1 = b^-1 a^-1
  const SE3 lhs = (a * b).inverse(), rhs = b.inverse() * a.inverse();
  CHECK_MAT_NEAR(lhs.t, rhs.t, 1e-12);
  // matrix() bottom row and consistency with the point action
  const Eigen::Matrix4d M = b.matrix();
  CHECK_MAT_NEAR(M.row(3), Eigen::RowVector4d(0, 0, 0, 1), 0.0);
  const Eigen::Vector4d ph = M * p.homogeneous();
  CHECK_MAT_NEAR(ph.head<3>().eval(), b * p, 1e-12);
  // default-constructed is identity
  SE3 id;
  CHECK_MAT_NEAR(id * p, p, 0.0);
  // pure translation inverse
  SE3 tr; tr.t = Eigen::Vector3d(1, 2, 3);
  CHECK_MAT_NEAR(tr.inverse().t, Eigen::Vector3d(-1, -2, -3), 0.0);
#endif
  return TEST_SUMMARY();
}
