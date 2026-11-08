// The candidate's code is injected before this file (g++ -include).
#include <limits>
#include "common/check.h"
#include "common/cloud_utils.h"

static double rotErr(const Eigen::Isometry3d& A, const Eigen::Isometry3d& B) {
  return Eigen::AngleAxisd(A.linear().transpose() * B.linear()).angle();
}
static double transErr(const Eigen::Isometry3d& A, const Eigen::Isometry3d& B) {
  return (A.translation() - B.translation()).norm();
}
// By value: CHECK_MAT_NEAR binds a const&, so never pass `.matrix()` of a temporary.
static Eigen::Matrix4d M(const Eigen::Isometry3d& T) { return T.matrix(); }
static const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();

int main() {
  const double deg = M_PI / 180.0;
  const cu::Cloud src = cu::uniformCube(250, 1.0);

  // Small motion (≈10°, 0.2 m) converges from identity.
  {
    const Eigen::Isometry3d Tt = cu::randomPose(10 * deg, 0.2);
    const cu::Cloud tgt = cu::transform(src, Tt);
    const Eigen::Isometry3d T = icp(src, tgt, I, 50);
    CHECK(rotErr(T, Tt) < 1e-6);
    CHECK(transErr(T, Tt) < 1e-6);
  }
  // src == tgt -> identity; max_iters = 0 -> init.
  {
    const Eigen::Isometry3d T = icp(src, src, I, 20);
    CHECK(rotErr(T, I) < 1e-9 && transErr(T, I) < 1e-9);
    const Eigen::Isometry3d init = cu::randomPose(0.1, 0.1);
    const Eigen::Isometry3d T0 = icp(src, src, init, 0);
    CHECK_MAT_NEAR(T0.matrix(), init.matrix(), 1e-9);
  }

#ifdef HIDDEN
  // Large rotation (90°) but a good init close to the truth: must start FROM init.
  {
    Eigen::Isometry3d Tt = Eigen::Isometry3d::Identity();
    Tt.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0.2, 0.3, 1).normalized()).toRotationMatrix();
    Tt.translation() = Eigen::Vector3d(2, -1, 0.5);
    const cu::Cloud tgt = cu::transform(src, Tt);
    const Eigen::Isometry3d init = Tt * cu::randomPose(5 * deg, 0.05);
    const Eigen::Isometry3d T = icp(src, tgt, init, 50);
    CHECK(rotErr(T, Tt) < 1e-6);
    CHECK(transErr(T, Tt) < 1e-6);
  }
  // Noisy target.
  {
    const Eigen::Isometry3d Tt = cu::randomPose(8 * deg, 0.15);
    cu::Cloud tgt = cu::transform(src, Tt);
    cu::addNoise(tgt, 0.003);
    const Eigen::Isometry3d T = icp(src, tgt, I, 50);
    CHECK(rotErr(T, Tt) < 0.01);
    CHECK(transErr(T, Tt) < 0.01);
    CHECK(std::abs(T.linear().determinant() - 1.0) < 1e-9);
  }
  // Partial overlap: the scan sees only part of the map, plus a dynamic object (clutter) that is
  // not in the map at all. Distance gating is what makes this converge exactly.
  {
    const Eigen::Isometry3d Tt = cu::randomPose(5 * deg, 0.1);
    const cu::Cloud tgt = cu::transform(src, Tt);  // "map"
    cu::Cloud scan;
    for (const auto& p : src) if (p.x() < 0.4) scan.push_back(p);
    for (const auto& c : cu::uniformCube(60, 0.5)) scan.push_back(c + Eigen::Vector3d(0, 0, 3));
    const Eigen::Isometry3d T = icp(scan, tgt, I, 60, 0.3);
    CHECK(rotErr(T, Tt) < 1e-6);
    CHECK(transErr(T, Tt) < 1e-6);
  }
  // Empty clouds -> init.
  {
    const Eigen::Isometry3d init = cu::randomPose(0.2, 0.3);
    CHECK_MAT_NEAR(M(icp({}, src, init, 10)), init.matrix(), 1e-9);
    CHECK_MAT_NEAR(M(icp(src, {}, init, 10)), init.matrix(), 1e-9);
  }
  // Gate so tight that < 3 pairs survive -> returns init unchanged.
  {
    const cu::Cloud far = cu::transform(src, Eigen::Isometry3d(Eigen::Translation3d(10, 0, 0)));
    CHECK_MAT_NEAR(M(icp(src, far, I, 10, 0.5)), I.matrix(), 1e-9);
  }
#endif
  return TEST_SUMMARY();
}
