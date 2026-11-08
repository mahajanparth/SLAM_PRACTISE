// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  // ground plane z = 0 in A; B = A shifted up by 2 m (x_B = x_A + (0,0,2)) -> plane z = 2 in B: (0,0,1,-2)
  Eigen::Isometry3d T_up = Eigen::Isometry3d::Identity();
  T_up.translation() = Eigen::Vector3d(0, 0, 2);
  CHECK_MAT_NEAR(transformPlane(T_up, Eigen::Vector4d(0, 0, 1, 0)), Eigen::Vector4d(0, 0, 1, -2), 1e-12);

  // pure rotation 90 deg about x: normal +z -> -y
  Eigen::Isometry3d T_rx = Eigen::Isometry3d::Identity();
  T_rx.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  CHECK_MAT_NEAR(transformPlane(T_rx, Eigen::Vector4d(0, 0, 1, -1)), Eigen::Vector4d(0, -1, 0, -1), 1e-12);

  // random pose: points on the plane stay on the plane
  const Eigen::Isometry3d T = cu::randomPose(M_PI, 5.0);
  const Eigen::Vector3d n = Eigen::Vector3d(1, -2, 0.5).normalized();
  const double d = 1.7;
  const Eigen::Vector4d pB = transformPlane(T, Eigen::Vector4d(n.x(), n.y(), n.z(), d));
  const cu::Cloud on = cu::planePoints(50, n, d, 3.0);
  double worst = 0;
  for (const auto& x : on) worst = std::max(worst, std::abs(pB.head<3>().dot(T * x) + pB(3)));
  CHECK(worst < 1e-10);

  // line
  Line3 L{Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(0, 0, 1)};
  Line3 LB = transformLine(T_up, L);
  CHECK_MAT_NEAR(LB.point, Eigen::Vector3d(1, 2, 5), 1e-12);
  CHECK_MAT_NEAR(LB.dir, Eigen::Vector3d(0, 0, 1), 1e-12);

#ifdef HIDDEN
  // identity
  const Eigen::Vector4d pl(0.3, -0.4, 0.5, 2.0);
  CHECK_MAT_NEAR(transformPlane(Eigen::Isometry3d::Identity(), pl), pl, 0.0);
  // scale is preserved (non-unit normal), result is linear in the plane vector
  const Eigen::Vector4d a = transformPlane(T, pl), b = transformPlane(T, (3.0 * pl).eval());
  CHECK_MAT_NEAR(b, (3.0 * a).eval(), 1e-12);
  CHECK_NEAR(a.head<3>().norm(), pl.head<3>().norm(), 1e-12);
  // matches the homogeneous formula pi_B = T^{-T} pi_A
  const Eigen::Vector4d ref = T.inverse().matrix().transpose() * pl;
  CHECK_MAT_NEAR(a, ref, 1e-10);
  // round trip A -> B -> A
  const Eigen::Vector4d back = transformPlane(T.inverse(), a);
  CHECK_MAT_NEAR(back, pl, 1e-10);
  // signed distances preserved (unit normal)
  const Eigen::Vector3d q(4, -1, 2);
  CHECK_NEAR(pB.head<3>().dot(T * q) + pB(3), n.dot(q) + d, 1e-10);
  // line under a random pose: transformed points of the line lie on the transformed line
  Line3 Lr{Eigen::Vector3d(-2, 1, 0.5), Eigen::Vector3d(1, 1, -2)};
  Line3 Lt = transformLine(T, Lr);
  const Eigen::Vector3d onB = T * (Lr.point + 2.5 * Lr.dir);
  CHECK(((onB - Lt.point).cross(Lt.dir)).norm() < 1e-10);
  CHECK_NEAR(Lt.dir.norm(), Lr.dir.norm(), 1e-12);
#endif
  return TEST_SUMMARY();
}
