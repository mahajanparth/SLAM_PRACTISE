// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"
#include "common/cloud_utils.h"

// Points on the 6 faces of the cube [-1,1]^3 with outward normals (all 6 DOF observable).
static void boxScene(int per_face, cu::Cloud& pts, cu::Cloud& nrm) {
  std::uniform_real_distribution<double> u(-1, 1);
  for (int axis = 0; axis < 3; ++axis)
    for (double s : {-1.0, 1.0})
      for (int k = 0; k < per_face; ++k) {
        Eigen::Vector3d p(u(cu::rng()), u(cu::rng()), u(cu::rng()));
        p[axis] = s;
        Eigen::Vector3d nn = Eigen::Vector3d::Zero();
        nn[axis] = s;
        pts.push_back(p);
        nrm.push_back(nn);
      }
}
// By value: CHECK_MAT_NEAR binds a const&, so never pass `.matrix()` of a temporary.
static Eigen::Matrix4d M(const Eigen::Isometry3d& T) { return T.matrix(); }
static double cost(const cu::Cloud& p, const cu::Cloud& q, const cu::Cloud& n, const Eigen::Isometry3d& T) {
  double c = 0;
  for (size_t i = 0; i < p.size(); ++i) { const double r = n[i].dot(T * p[i] - q[i]); c += r * r; }
  return c;
}
// Target = Tt * source; returns the estimate after `iters` GN steps with fixed correspondences.
static Eigen::Isometry3d iterate(const cu::Cloud& p0, const cu::Cloud& q, const cu::Cloud& n, int iters) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int k = 0; k < iters; ++k) T = applyUpdate(pointToPlaneStep(cu::transform(p0, T), q, n), T);
  return T;
}

int main() {
  cu::Cloud p0, n0;
  boxScene(30, p0, n0);
  const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();

  // Pure translation: the problem is linear, so one step is exact.
  {
    const Eigen::Vector3d v(0.05, -0.02, 0.03);
    cu::Cloud q;
    for (const auto& x : p0) q.push_back(x + v);
    const Vector6d d = pointToPlaneStep(p0, q, n0);
    CHECK(d.head<3>().norm() < 1e-9);
    CHECK_MAT_NEAR(d.tail<3>(), v, 1e-9);
  }
  // Small rotation + translation: one step gets close, iterating converges.
  {
    const Eigen::Isometry3d Tt = cu::randomPose(2 * M_PI / 180, 0.05);
    const cu::Cloud q = cu::transform(p0, Tt);
    cu::Cloud n;
    for (const auto& x : n0) n.push_back(Tt.linear() * x);
    const Vector6d d = pointToPlaneStep(p0, q, n);
    const Eigen::AngleAxisd aa(Tt.linear());
    CHECK_MAT_NEAR(d.head<3>(), aa.angle() * aa.axis(), 2e-3);
    const Eigen::Isometry3d T1 = applyUpdate(d, I);
    CHECK(cost(p0, q, n, T1) < 1e-2 * cost(p0, q, n, I));
    const Eigen::Isometry3d T = iterate(p0, q, n, 6);
    CHECK_MAT_NEAR(T.matrix(), Tt.matrix(), 1e-9);
  }

#ifdef HIDDEN
  // applyUpdate semantics: zero -> unchanged; rotation applied on the LEFT (rotates the translation too).
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(1, 0, 0);
    CHECK_MAT_NEAR(M(applyUpdate(Vector6d::Zero(), T)), T.matrix(), 1e-15);
    Vector6d d = Vector6d::Zero();
    d(2) = M_PI / 2;   // 90° about z
    d(3) = 0.5;        // + translation x
    const Eigen::Isometry3d U = applyUpdate(d, T);
    CHECK_MAT_NEAR(U.translation(), Eigen::Vector3d(0.5, 1, 0), 1e-12);
    CHECK_MAT_NEAR(U.linear(), Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix(), 1e-12);
    CHECK(std::abs(U.linear().determinant() - 1.0) < 1e-12);
  }
  // Single plane: rotations about its normal & in-plane translations unobservable -> zero.
  {
    cu::Cloud p = cu::planePoints(100, {0, 0, 1}, 0.0, 2.0), q, n;
    for (const auto& x : p) { q.push_back(x + Eigen::Vector3d(0.1, 0.1, 0.1)); n.push_back(Eigen::Vector3d::UnitZ()); }
    CHECK(pointToPlaneStep(p, q, n).norm() == 0.0);
  }
  // Too few / mismatched / empty.
  {
    cu::Cloud p(p0.begin(), p0.begin() + 5), n(n0.begin(), n0.begin() + 5);
    CHECK(pointToPlaneStep(p, p, n).norm() == 0.0);
    CHECK(pointToPlaneStep(p0, p0, n).norm() == 0.0);
    CHECK(pointToPlaneStep({}, {}, {}).norm() == 0.0);
  }
  // Larger motion (10°, 0.2 m), tilted extra planes, converges in 10 steps.
  {
    cu::Cloud p = p0, nn = n0;
    const Eigen::Vector3d tn = Eigen::Vector3d(1, 1, 1).normalized();
    for (const auto& x : cu::planePoints(40, tn, -0.5, 1.0)) { p.push_back(x); nn.push_back(tn); }
    const Eigen::Isometry3d Tt = cu::randomPose(10 * M_PI / 180, 0.2);
    const cu::Cloud q = cu::transform(p, Tt);
    cu::Cloud n;
    for (const auto& x : nn) n.push_back(Tt.linear() * x);
    CHECK_MAT_NEAR(M(iterate(p, q, n, 10)), Tt.matrix(), 1e-9);
    // Already aligned -> zero step.
    CHECK(pointToPlaneStep(q, q, n).norm() < 1e-12);
  }
#endif
  return TEST_SUMMARY();
}
