// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  // plane z = 0 seen from above -> +z; from below -> -z
  const cu::Cloud plane = cu::planePoints(200, Eigen::Vector3d::UnitZ(), 0.0, 2.0);
  auto up = estimateNormals(plane, 10, Eigen::Vector3d(0, 0, 10));
  auto down = estimateNormals(plane, 10, Eigen::Vector3d(0, 0, -10));
  CHECK(up.size() == plane.size() && down.size() == plane.size());
  bool ok_up = up.size() == plane.size(), ok_down = ok_up, curv = ok_up;
  for (size_t i = 0; ok_up && i < up.size(); ++i) {
    ok_up &= up[i].valid && (up[i].n - Eigen::Vector3d::UnitZ()).norm() < 1e-6;
    ok_down &= down[i].valid && (down[i].n + Eigen::Vector3d::UnitZ()).norm() < 1e-6;
    curv &= up[i].curvature < 1e-9;
  }
  CHECK(ok_up);
  CHECK(ok_down);
  CHECK(curv);

  // sphere with the viewpoint at its centre -> normals point inward, along (c - p) / r
  const Eigen::Vector3d ctr(1, -2, 0.5);
  const cu::Cloud sph = cu::spherePoints(400, ctr, 2.0);
  auto ns = estimateNormals(sph, 8, ctr);
  bool ok_s = ns.size() == sph.size();
  for (size_t i = 0; ok_s && i < ns.size(); ++i)
    ok_s &= ns[i].valid && ns[i].n.dot((ctr - sph[i]).normalized()) > 0.97 && std::abs(ns[i].n.norm() - 1) < 1e-9;
  CHECK(ok_s);

#ifdef HIDDEN
  // fewer than 3 neighbours
  auto two = estimateNormals({{0, 0, 0}, {1, 0, 0}}, 5, Eigen::Vector3d::Zero());
  CHECK(two.size() == 2 && !two[0].valid && !two[1].valid);
  auto k2 = estimateNormals(plane, 2, Eigen::Vector3d(0, 0, 1));
  CHECK(k2.size() == plane.size() && !k2[0].valid);
  // collinear points -> invalid
  cu::Cloud line;
  for (int i = 0; i < 20; ++i) line.push_back(Eigen::Vector3d(0.1 * i, 0.2 * i, -0.05 * i));
  auto nl = estimateNormals(line, 6, Eigen::Vector3d(5, 0, 0));
  bool all_invalid = nl.size() == line.size();
  for (const auto& x : nl) all_invalid &= !x.valid;
  CHECK(all_invalid);
  // identical points -> invalid
  auto same = estimateNormals(cu::Cloud(5, Eigen::Vector3d(1, 1, 1)), 4, Eigen::Vector3d::Zero());
  CHECK(same.size() == 5 && !same[0].valid);
  // tilted plane far from the origin (two-pass covariance needed)
  const Eigen::Vector3d nt = Eigen::Vector3d(0.3, -0.2, 1.0).normalized();
  cu::Cloud farp = cu::planePoints(300, nt, 0.0, 2.0);
  const Eigen::Vector3d off(4.0e6, -3.0e6, 0.0);
  const Eigen::Vector3d offp = off - nt * nt.dot(off) + nt * 1e6;  // keep points on a plane with normal nt
  for (auto& p : farp) p += offp;
  auto nf = estimateNormals(farp, 10, offp + 100 * nt);
  bool ok_far = nf.size() == farp.size();
  for (size_t i = 0; ok_far && i < nf.size(); ++i) ok_far &= nf[i].valid && (nf[i].n - nt).norm() < 1e-4;
  CHECK(ok_far);
  // a box corner has high curvature compared to a flat patch
  cu::Cloud corner;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      corner.push_back(Eigen::Vector3d(0.1 * i, 0.1 * j, 0));
      corner.push_back(Eigen::Vector3d(0.1 * i, 0, 0.1 * (j + 1)));
      corner.push_back(Eigen::Vector3d(0, 0.1 * (i + 1), 0.1 * (j + 1)));
    }
  auto nc = estimateNormals(corner, 15, Eigen::Vector3d(1, 1, 1));
  CHECK(nc.size() == corner.size() && nc[0].valid && nc[0].curvature > 0.05);
  // empty cloud
  CHECK(estimateNormals({}, 5, Eigen::Vector3d::Zero()).empty());
#endif
  return TEST_SUMMARY();
}
