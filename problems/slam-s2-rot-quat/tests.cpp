// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

// Same rotation? (compare up to the double cover)
static bool sameQuat(const Eigen::Vector4d& q, const Eigen::Quaterniond& ref, double tol) {
  Eigen::Vector4d r(ref.w(), ref.x(), ref.y(), ref.z());
  return std::abs(q.norm() - 1.0) < tol && std::abs(std::abs(q.dot(r.normalized())) - 1.0) < tol;
}

int main() {
  // quatToRot vs Eigen
  Eigen::Quaterniond qa = Eigen::Quaterniond(0.9, 0.1, -0.3, 0.2).normalized();
  CHECK_MAT_NEAR(quatToRot(Eigen::Vector4d(qa.w(), qa.x(), qa.y(), qa.z())), qa.toRotationMatrix(), 1e-12);

  // rotToQuat on a generic rotation
  Eigen::Matrix3d R1 = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  Eigen::Vector4d q1 = rotToQuat(R1);
  CHECK(sameQuat(q1, Eigen::Quaterniond(R1), 1e-12));
  CHECK(q1(0) >= 0);
  CHECK_MAT_NEAR(quatToRot(q1), R1, 1e-12);

  // identity
  CHECK_MAT_NEAR(rotToQuat(Eigen::Matrix3d::Identity()), Eigen::Vector4d(1, 0, 0, 0), 1e-15);

  // round trip on random rotations
  bool rt = true;
  for (int i = 0; i < 50; ++i) {
    Eigen::Matrix3d R = cu::randomPose(M_PI, 0).linear();
    rt &= (quatToRot(rotToQuat(R)) - R).norm() < 1e-10;
    rt &= sameQuat(rotToQuat(R), Eigen::Quaterniond(R), 1e-10);
  }
  CHECK(rt);

#ifdef HIDDEN
  // 180 deg about each axis: trace = -1, w = 0 -> naive formula divides by 0
  const Eigen::Vector3d axes[] = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(),
                                  Eigen::Vector3d(1, -1, 2).normalized(), Eigen::Vector3d(-3, 1, 0.5).normalized()};
  for (const auto& ax : axes) {
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI, ax).toRotationMatrix();
    Eigen::Vector4d q = rotToQuat(R);
    CHECK(q.allFinite());
    CHECK_MAT_NEAR(quatToRot(q), R, 1e-10);
  }
  // near 180 deg (trace slightly > -1) and w >= 0 canonicalization
  Eigen::Matrix3d Rn = Eigen::AngleAxisd(M_PI - 1e-6, Eigen::Vector3d(0, 1, 1).normalized()).toRotationMatrix();
  CHECK(sameQuat(rotToQuat(Rn), Eigen::Quaterniond(Rn), 1e-9));
  Eigen::Matrix3d Rneg = Eigen::AngleAxisd(-2.5, Eigen::Vector3d(0.2, -0.4, 1).normalized()).toRotationMatrix();
  Eigen::Vector4d qneg = rotToQuat(Rneg);
  CHECK(qneg(0) >= 0);
  CHECK(sameQuat(qneg, Eigen::Quaterniond(Rneg), 1e-12));
  // non-normalized quaternion input
  Eigen::Vector4d big(3.0, -1.0, 2.0, 0.5);
  Eigen::Quaterniond qb(big(0), big(1), big(2), big(3));
  CHECK_MAT_NEAR(quatToRot(big), qb.normalized().toRotationMatrix(), 1e-12);
  // q and -q give the same R
  CHECK_MAT_NEAR(quatToRot(-big), quatToRot(big), 1e-12);
  // tiny rotation
  Eigen::Matrix3d Rt = Eigen::AngleAxisd(1e-9, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  CHECK(sameQuat(rotToQuat(Rt), Eigen::Quaterniond(Rt), 1e-14));
#endif
  return TEST_SUMMARY();
}
