// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static Eigen::Isometry3d make(double yaw, const Eigen::Vector3d& t) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = t;
  return T;
}

int main() {
  // Hand-checkable: robot at (10,0,0) facing +y; lidar 1 m forward on the robot; camera at world origin, identity.
  const Eigen::Isometry3d T_w_r = make(M_PI / 2, {10, 0, 0});
  const Eigen::Isometry3d T_r_l = make(0, {1, 0, 0});
  const Eigen::Isometry3d T_w_c = Eigen::Isometry3d::Identity();
  // lidar point 2 m ahead of the lidar -> 3 m ahead of the robot -> world (10, 3, 0)
  CHECK_MAT_NEAR(lidarPointInCamera(T_w_r, T_r_l, T_w_c, Eigen::Vector3d(2, 0, 0)), Eigen::Vector3d(10, 3, 0), 1e-12);
  // camera at (10, 5, 0) facing -y (yaw = -90 deg): world (10,3,0) is 2 m in front -> camera (2, 0, 0)
  CHECK_MAT_NEAR(lidarPointInCamera(T_w_r, T_r_l, make(-M_PI / 2, {10, 5, 0}), Eigen::Vector3d(2, 0, 0)),
                 Eigen::Vector3d(2, 0, 0), 1e-12);

  // relative pose: a at (1,0,0) yaw 90deg, b at (1,2,0) yaw 90deg -> b is 2 m straight ahead of a
  const Eigen::Isometry3d T_w_a = make(M_PI / 2, {1, 0, 0}), T_w_b = make(M_PI / 2, {1, 2, 0});
  const Eigen::Isometry3d T_a_b = relativePose(T_w_a, T_w_b);
  CHECK_MAT_NEAR(T_a_b.translation(), Eigen::Vector3d(2, 0, 0), 1e-12);
  CHECK_MAT_NEAR(T_a_b.linear(), Eigen::Matrix3d::Identity(), 1e-12);
  const Eigen::Isometry3d pred = predictPose(T_w_a, T_a_b);
  CHECK_MAT_NEAR(pred.matrix(), T_w_b.matrix(), 1e-12);

#ifdef HIDDEN
  // random full-3D poses: catches any swapped order
  bool ok = true;
  for (int i = 0; i < 20; ++i) {
    const Eigen::Isometry3d A = cu::randomPose(M_PI, 5), B = cu::randomPose(M_PI, 5), C = cu::randomPose(M_PI, 5);
    const Eigen::Vector3d p = Eigen::Vector3d::Random() * 20;
    const Eigen::Vector3d ref = C.inverse() * (A * (B * p));
    ok &= (lidarPointInCamera(A, B, C, p) - ref).norm() < 1e-10;
    const Eigen::Isometry3d rel = relativePose(A, B);
    ok &= ((A * rel).matrix() - B.matrix()).norm() < 1e-10;
    ok &= (predictPose(A, rel).matrix() - B.matrix()).norm() < 1e-10;
    ok &= (predictPose(A, C).matrix() - (A * C).matrix()).norm() < 1e-10;
  }
  CHECK(ok);
  // relative pose to itself is identity
  const Eigen::Isometry3d R0 = cu::randomPose(M_PI, 3);
  const Eigen::Matrix4d self = relativePose(R0, R0).matrix();
  CHECK_MAT_NEAR(self, Eigen::Matrix4d::Identity(), 1e-12);
  // relative pose is NOT the world-frame difference T_w_b * T_w_a^-1
  const Eigen::Isometry3d Ta = make(M_PI / 2, {5, 5, 0}), Tb = make(M_PI, {5, 7, 0});
  const Eigen::Isometry3d rab = relativePose(Ta, Tb);
  CHECK_MAT_NEAR(rab.translation(), Eigen::Vector3d(2, 0, 0), 1e-12);
  // inverse relationship
  const Eigen::Matrix4d rba = relativePose(Tb, Ta).matrix(), rab_inv = rab.inverse().matrix();
  CHECK_MAT_NEAR(rba, rab_inv, 1e-12);
  // result is a proper rotation
  const Eigen::Matrix3d Rr = relativePose(cu::randomPose(M_PI, 1), cu::randomPose(M_PI, 1)).linear();
  CHECK_MAT_NEAR((Rr.transpose() * Rr).eval(), Eigen::Matrix3d::Identity(), 1e-12);
#endif
  return TEST_SUMMARY();
}
