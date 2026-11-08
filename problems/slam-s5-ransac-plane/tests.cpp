// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <vector>
#include "common/check.h"
#include "common/cloud_utils.h"

// |cos angle| between normals, and offset compared with the matching sign.
static bool planeClose(const Eigen::Vector4d& pl, Eigen::Vector3d n, double d, double ang_tol, double d_tol) {
  n.normalize();  // d is w.r.t. the unit normal (as in cu::planePoints)
  const Eigen::Vector3d m = pl.head<3>();
  if (std::abs(m.norm() - 1.0) > 1e-6) return false;
  const double sign = m.dot(n) >= 0 ? 1.0 : -1.0;
  return std::abs(m.dot(n)) >= std::cos(ang_tol) && std::abs(sign * pl[3] - d) <= d_tol;
}
static bool inliersConsistent(const cu::Cloud& c, const Eigen::Vector4d& pl, const std::vector<int>& in, double th) {
  if (!std::is_sorted(in.begin(), in.end())) return false;
  size_t cnt = 0;
  for (size_t i = 0; i < c.size(); ++i)
    if (std::abs(pl.head<3>().dot(c[i]) + pl[3]) <= th + 1e-12) ++cnt;
  if (cnt != in.size()) return false;
  for (int i : in)
    if (i < 0 || i >= static_cast<int>(c.size()) || std::abs(pl.head<3>().dot(c[i]) + pl[3]) > th + 1e-12) return false;
  return true;
}

int main() {
  // Ground plane z = 1 (n=(0,0,1), d=-1): 700 inliers + 300 clutter points.
  cu::Cloud c = cu::planePoints(700, {0, 0, 1}, -1.0, 5.0, 0.01);
  cu::Cloud clutter = cu::uniformCube(300, 5.0);
  c.insert(c.end(), clutter.begin(), clutter.end());
  Eigen::Vector4d pl = Eigen::Vector4d::Zero();
  std::vector<int> in;
  const bool ok = ransacPlane(c, 0.05, 200, pl, in);
  CHECK(ok);
  CHECK(planeClose(pl, {0, 0, 1}, -1.0, 0.01, 0.01));
  CHECK(in.size() >= 690 && in.size() <= 730);
  CHECK(inliersConsistent(c, pl, in, 0.05));

  // Too few points.
  Eigen::Vector4d pl2 = Eigen::Vector4d::Zero();
  std::vector<int> in2;
  CHECK(!ransacPlane(cu::Cloud{{0, 0, 0}, {1, 0, 0}}, 0.1, 50, pl2, in2));

#ifdef HIDDEN
  // Tilted plane with 30% outliers.
  {
    const Eigen::Vector3d n(1, -2, 0.5);
    cu::Cloud t = cu::planePoints(700, n, 2.0, 3.0, 0.005);
    cu::Cloud o = cu::uniformCube(300, 4.0);
    t.insert(t.end(), o.begin(), o.end());
    Eigen::Vector4d p = Eigen::Vector4d::Zero();
    std::vector<int> i;
    CHECK(ransacPlane(t, 0.03, 300, p, i));
    CHECK(planeClose(p, n, 2.0, 0.01, 0.02));
    CHECK(i.size() >= 690 && i.size() <= 730);
    CHECK(inliersConsistent(t, p, i, 0.03));
    // Repeatable with the same seed.
    Eigen::Vector4d p2 = Eigen::Vector4d::Zero();
    std::vector<int> i2;
    ransacPlane(t, 0.03, 300, p2, i2);
    CHECK(i == i2);
  }
  // All collinear points -> no plane.
  {
    cu::Cloud line;
    for (int k = 0; k < 50; ++k) line.push_back(Eigen::Vector3d(k, 2 * k, -3 * k));
    Eigen::Vector4d p = Eigen::Vector4d::Zero();
    std::vector<int> i;
    CHECK(!ransacPlane(line, 0.01, 100, p, i));
  }
  // All identical points -> no plane; empty cloud -> false.
  {
    Eigen::Vector4d p = Eigen::Vector4d::Zero();
    std::vector<int> i;
    CHECK(!ransacPlane(cu::Cloud(20, Eigen::Vector3d(1, 1, 1)), 0.01, 100, p, i));
    CHECK(!ransacPlane(cu::Cloud{}, 0.01, 100, p, i));
  }
  // Exactly 3 points -> the plane through them, all inliers.
  {
    cu::Cloud three = {{0, 0, 2}, {1, 0, 2}, {0, 1, 2}};
    Eigen::Vector4d p = Eigen::Vector4d::Zero();
    std::vector<int> i;
    CHECK(ransacPlane(three, 1e-6, 100, p, i));
    CHECK(planeClose(p, {0, 0, 1}, -2.0, 1e-6, 1e-6));
    CHECK(i == std::vector<int>({0, 1, 2}));
  }
#endif
  return TEST_SUMMARY();
}
