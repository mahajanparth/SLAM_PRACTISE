// The candidate's code is injected before this file (g++ -include).
#include <vector>
#include "common/check.h"
#include "common/cloud_utils.h"

static double lineDist(const Eigen::Vector3d& x, const Eigen::Vector3d& a, const Eigen::Vector3d& u) {
  return ((x - a) - (x - a).dot(u) * u).norm();
}

int main() {
  // Vertical plane x = 2 (z = ax+by+c cannot represent it).
  {
    cu::Cloud c = cu::planePoints(200, {1, 0, 0}, -2.0, 3.0, 0.0);
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    double d = 0;
    CHECK(fitPlane(c, n, d));
    CHECK_NEAR(std::abs(n.x()), 1.0, 1e-9);
    CHECK_NEAR(n.norm(), 1.0, 1e-9);
    CHECK_NEAR(n.x() > 0 ? d : -d, -2.0, 1e-9);
  }
  // 3D line through (1,2,3) along (1,1,0)/sqrt2.
  {
    const Eigen::Vector3d a(1, 2, 3), u = Eigen::Vector3d(1, 1, 0).normalized();
    cu::Cloud c;
    for (int k = -10; k <= 10; ++k) c.push_back(a + 0.3 * k * u);
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), dir = Eigen::Vector3d::Zero();
    CHECK(fitLine(c, p, dir));
    CHECK_NEAR(std::abs(dir.dot(u)), 1.0, 1e-9);
    CHECK_NEAR(lineDist(p, a, u), 0.0, 1e-9);
  }

#ifdef HIDDEN
  // Noisy tilted plane, minimises orthogonal error.
  {
    const Eigen::Vector3d nt = Eigen::Vector3d(0.3, -0.5, 1).normalized();
    cu::Cloud c = cu::planePoints(1000, nt, 1.5, 4.0, 0.01);
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    double d = 0;
    CHECK(fitPlane(c, n, d));
    CHECK(std::abs(n.dot(nt)) > std::cos(0.005));
    CHECK_NEAR(n.dot(nt) > 0 ? d : -d, 1.5, 0.01);
  }
  // Exactly 3 points.
  {
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    double d = 0;
    CHECK(fitPlane({{0, 0, -1}, {3, 0, -1}, {0, 5, -1}}, n, d));
    CHECK_NEAR(std::abs(n.z()), 1.0, 1e-9);
  }
  // Degenerate inputs.
  {
    Eigen::Vector3d n, p, u;
    double d;
    CHECK(!fitPlane({{0, 0, 0}, {1, 1, 1}}, n, d));
    CHECK(!fitPlane({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {-3, -3, -3}}, n, d));  // collinear
    CHECK(!fitLine({{4, 5, 6}}, p, u));
    CHECK(!fitLine(cu::Cloud(10, Eigen::Vector3d(4, 5, 6)), p, u));
    CHECK(!fitLine({}, p, u));
  }
  // Far from origin (map coords ~1e4 m): must centre before accumulating.
  {
    const Eigen::Vector3d off(1e4, -2e4, 3e3);
    cu::Cloud c = cu::planePoints(500, {0, 0, 1}, 0.0, 0.5, 0.0);
    for (auto& q : c) q += off;
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    double d = 0;
    CHECK(fitPlane(c, n, d));
    CHECK_NEAR(std::abs(n.z()), 1.0, 1e-6);
    // Line with large offset.
    const Eigen::Vector3d u = Eigen::Vector3d(0, 0.6, 0.8);
    cu::Cloud l;
    for (int k = 0; k < 50; ++k) l.push_back(off + 0.01 * k * u);
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), dir = Eigen::Vector3d::Zero();
    CHECK(fitLine(l, p, dir));
    CHECK_NEAR(std::abs(dir.dot(u)), 1.0, 1e-6);
  }
#endif
  return TEST_SUMMARY();
}
