// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  const Eigen::Isometry3d T = cu::randomPose(M_PI, 10.0);
  const cu::Cloud pts = cu::uniformCube(2000, 50.0);
  const Eigen::Matrix3Xd P = cu::toMatrix(pts);

  Eigen::Matrix3Xd Q = transformCloud(T, P);
  CHECK(Q.cols() == P.cols());
  if (Q.cols() == P.cols()) {
    CHECK_MAT_NEAR(Q.col(0), T * pts[0], 1e-12);
    CHECK_MAT_NEAR(Q, cu::toMatrix(cu::transform(pts, T)), 1e-10);
  }
  std::vector<Eigen::Vector3d> q = transformCloud(T, pts);
  CHECK(q.size() == pts.size());
  if (q.size() == pts.size()) CHECK_MAT_NEAR(cu::toMatrix(q), cu::toMatrix(cu::transform(pts, T)), 1e-10);

  // raw buffer x,y,z,intensity
  std::vector<float> buf;
  for (const auto& p : pts) { buf.push_back(float(p.x())); buf.push_back(float(p.y())); buf.push_back(float(p.z())); buf.push_back(123.f); }
  Eigen::Matrix3Xf R = transformRaw(T, buf.data(), int(pts.size()));
  CHECK(R.cols() == int(pts.size()));
  if (R.cols() == int(pts.size())) {
    Eigen::Matrix3Xf ref(3, pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
      ref.col(i) = (T * Eigen::Vector3d(buf[4 * i], buf[4 * i + 1], buf[4 * i + 2])).cast<float>();
    CHECK_MAT_NEAR(R, ref, 1e-4);
  }

#ifdef HIDDEN
  // empty inputs
  CHECK(transformCloud(T, Eigen::Matrix3Xd(3, 0)).cols() == 0);
  CHECK(transformCloud(T, std::vector<Eigen::Vector3d>{}).empty());
  CHECK(transformRaw(T, buf.data(), 0).cols() == 0);
  // identity leaves points unchanged; pure translation
  CHECK_MAT_NEAR(transformCloud(Eigen::Isometry3d::Identity(), P), P, 0.0);
  Eigen::Isometry3d Tt = Eigen::Isometry3d::Identity();
  Tt.translation() = Eigen::Vector3d(1, -2, 3);
  Eigen::Matrix3Xd Pt = transformCloud(Tt, P);
  if (Pt.cols() == P.cols()) CHECK_MAT_NEAR((Pt - P).rowwise().mean().eval(), Eigen::Vector3d(1, -2, 3), 1e-12);
  else CHECK(false);
  // single point
  std::vector<Eigen::Vector3d> one = transformCloud(T, std::vector<Eigen::Vector3d>{Eigen::Vector3d(1, 2, 3)});
  CHECK(one.size() == 1);
  if (one.size() == 1) CHECK_MAT_NEAR(one[0], T * Eigen::Vector3d(1, 2, 3), 1e-12);
  // raw: intensity must be ignored even if huge / NaN-free garbage
  std::vector<float> b2 = {1, 0, 0, 1e30f, 0, 1, 0, -1e30f};
  Eigen::Matrix3Xf r2 = transformRaw(T, b2.data(), 2);
  CHECK(r2.cols() == 2);
  if (r2.cols() == 2) CHECK_MAT_NEAR(r2.col(1), (T * Eigen::Vector3d(0, 1, 0)).cast<float>().eval(), 1e-5);
  // larger cloud (100k) both layouts agree
  const cu::Cloud big = cu::uniformCube(100000, 100.0);
  Eigen::Matrix3Xd QB = transformCloud(T, cu::toMatrix(big));
  std::vector<Eigen::Vector3d> qb = transformCloud(T, big);
  CHECK(QB.cols() == 100000 && qb.size() == 100000);
  if (QB.cols() == 100000 && qb.size() == 100000) CHECK_MAT_NEAR(QB, cu::toMatrix(qb), 1e-10);
#endif
  return TEST_SUMMARY();
}
