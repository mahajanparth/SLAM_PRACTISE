// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static bool isRotation(const Eigen::Matrix3d& R) {
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-9 &&
         std::abs(R.determinant() - 1.0) < 1e-9;
}
// By value: CHECK_MAT_NEAR binds a const&, so never pass `.matrix()` of a temporary.
static Eigen::Matrix4d M(const Eigen::Isometry3d& T) { return T.matrix(); }
static Eigen::Matrix3Xd apply(const Eigen::Isometry3d& T, const Eigen::Matrix3Xd& P) {
  return (T.linear() * P).colwise() + T.translation();
}

int main() {
  // Noise-free random pose.
  const Eigen::Matrix3Xd P = cu::toMatrix(cu::uniformCube(50, 2.0));
  const Eigen::Isometry3d Tt = cu::randomPose(M_PI, 3.0);
  const Eigen::Matrix3Xd Q = apply(Tt, P);
  const Eigen::Isometry3d T = kabsch(P, Q);
  CHECK(isRotation(T.linear()));
  CHECK_MAT_NEAR(T.matrix(), Tt.matrix(), 1e-9);

  // Invalid inputs -> identity.
  CHECK_MAT_NEAR(M(kabsch(P, Q.leftCols(10))), Eigen::Matrix4d::Identity(), 0.0);

  // Umeyama with scale 2.5.
  {
    Eigen::Matrix3Xd Qs = (2.5 * Tt.linear() * P).colwise() + Tt.translation();
    Eigen::Matrix4d S = umeyamaSim3(P, Qs);
    Eigen::Matrix4d St = Eigen::Matrix4d::Identity();
    St.topLeftCorner<3, 3>() = 2.5 * Tt.linear();
    St.topRightCorner<3, 1>() = Tt.translation();
    CHECK_MAT_NEAR(S, St, 1e-9);
  }

#ifdef HIDDEN
  // Mirror image: the best unconstrained map is a reflection; we need the best PROPER rotation.
  {
    Eigen::Matrix3Xd Qm = P;
    Qm.row(0) *= -1.0;
    Qm.colwise() += Eigen::Vector3d(1, 2, 3);
    const Eigen::Isometry3d Tm = kabsch(P, Qm);
    CHECK(isRotation(Tm.linear()));
    CHECK_MAT_NEAR(Tm.matrix(), Eigen::Matrix4d(Eigen::umeyama(P, Qm, false)), 1e-8);
  }
  // Planar points (rank-2 H): naive V Uᵀ may be a reflection.
  {
    const Eigen::Matrix3Xd Pp = cu::toMatrix(cu::planePoints(30, {0, 0, 1}, 0.0, 2.0));
    for (int k = 0; k < 4; ++k) {
      const Eigen::Isometry3d Tp = cu::randomPose(M_PI, 2.0);
      CHECK_MAT_NEAR(M(kabsch(Pp, apply(Tp, Pp))), Tp.matrix(), 1e-8);
    }
  }
  // Exactly 3 points; fewer than 3 -> identity.
  {
    const Eigen::Matrix3Xd P3 = P.leftCols(3);
    CHECK_MAT_NEAR(M(kabsch(P3, apply(Tt, P3))), Tt.matrix(), 1e-8);
    CHECK_MAT_NEAR(M(kabsch(P.leftCols(2), Q.leftCols(2))), Eigen::Matrix4d::Identity(), 0.0);
    CHECK_MAT_NEAR(umeyamaSim3(P.leftCols(2), Q.leftCols(2)), Eigen::Matrix4d::Identity(), 0.0);
  }
  // Collinear points: rotation not unique, but must be proper and achieve zero residual.
  {
    Eigen::Matrix3Xd L(3, 10);
    for (int k = 0; k < 10; ++k) L.col(k) = Eigen::Vector3d(1, -2, 0.5) * (k - 4.5) + Eigen::Vector3d(3, 0, 1);
    const Eigen::Matrix3Xd QL = apply(Tt, L);
    const Eigen::Isometry3d TL = kabsch(L, QL);
    CHECK(isRotation(TL.linear()));
    CHECK((apply(TL, L) - QL).cwiseAbs().maxCoeff() < 1e-8);
  }
  // Noisy data: matches Eigen::umeyama (rigid and similarity).
  {
    cu::Cloud src = cu::uniformCube(200, 3.0);
    const Eigen::Isometry3d Tn = cu::randomPose(M_PI, 5.0);
    cu::Cloud dst = cu::transform(src, Tn);
    cu::addNoise(dst, 0.05);
    const Eigen::Matrix3Xd A = cu::toMatrix(src), B = cu::toMatrix(dst);
    CHECK_MAT_NEAR(M(kabsch(A, B)), Eigen::Matrix4d(Eigen::umeyama(A, B, false)), 1e-8);
    const Eigen::Matrix3Xd Bs = 0.3 * B;
    CHECK_MAT_NEAR(umeyamaSim3(A, Bs), Eigen::Matrix4d(Eigen::umeyama(A, Bs, true)), 1e-8);
  }
#endif
  return TEST_SUMMARY();
}
