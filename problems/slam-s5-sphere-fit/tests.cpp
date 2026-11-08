// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <vector>
#include "common/check.h"
#include "common/cloud_utils.h"

int main() {
  // Noise-free sphere.
  {
    const Eigen::Vector3d c0(1, -2, 3);
    cu::Cloud s = cu::spherePoints(200, c0, 2.5);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    double r = 0;
    CHECK(fitSphere(s, c, r));
    CHECK_MAT_NEAR(c, c0, 1e-9);
    CHECK_NEAR(r, 2.5, 1e-9);
  }
  // Too few points.
  {
    Eigen::Vector3d c;
    double r;
    CHECK(!fitSphere({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, c, r));
  }
  // RANSAC: 300 on the sphere + 200 clutter.
  {
    const Eigen::Vector3d c0(-0.5, 0.5, 1.0);
    cu::Cloud s = cu::spherePoints(300, c0, 1.2);
    cu::addNoise(s, 0.005);
    cu::Cloud o = cu::uniformCube(200, 4.0);
    s.insert(s.end(), o.begin(), o.end());
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    double r = 0;
    std::vector<int> in;
    CHECK(ransacSphere(s, 0.03, 300, c, r, in));
    CHECK_MAT_NEAR(c, c0, 0.01);
    CHECK_NEAR(r, 1.2, 0.01);
    CHECK(in.size() >= 300 && in.size() <= 330);
  }

#ifdef HIDDEN
  // Noisy partial cap (upper hemisphere only), far from the origin.
  {
    const Eigen::Vector3d c0(1000, 2000, -500);
    cu::Cloud s = cu::spherePoints(600, c0, 1.5), cap;
    for (const auto& p : s) if (p.z() > c0.z() + 0.3) cap.push_back(p);
    cu::addNoise(cap, 0.002);
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    double r = 0;
    CHECK(fitSphere(cap, c, r));
    CHECK_MAT_NEAR(c, c0, 0.03);
    CHECK_NEAR(r, 1.5, 0.03);
  }
  // Exactly 4 non-coplanar points: exact sphere.
  {
    const Eigen::Vector3d c0(0, 0, 0);
    Eigen::Vector3d c = Eigen::Vector3d::Ones();
    double r = 0;
    CHECK(fitSphere({{2, 0, 0}, {0, 2, 0}, {0, 0, 2}, {-2, 0, 0}}, c, r));
    CHECK_MAT_NEAR(c, c0, 1e-9);
    CHECK_NEAR(r, 2.0, 1e-9);
  }
  // Coplanar points (a circle) -> degenerate.
  {
    cu::Cloud circ;
    for (int k = 0; k < 20; ++k) circ.push_back({std::cos(0.3 * k), std::sin(0.3 * k), 0.0});
    Eigen::Vector3d c;
    double r;
    CHECK(!fitSphere(circ, c, r));
  }
  // RANSAC degenerate inputs + determinism.
  {
    Eigen::Vector3d c;
    double r;
    std::vector<int> in;
    CHECK(!ransacSphere({{0, 0, 0}, {1, 1, 1}, {2, 0, 1}}, 0.1, 50, c, r, in));
    cu::Cloud s = cu::spherePoints(150, {3, 3, 3}, 0.7);
    cu::Cloud o = cu::uniformCube(150, 5.0);
    s.insert(s.end(), o.begin(), o.end());
    std::vector<int> a, b;
    Eigen::Vector3d c1, c2;
    double r1 = 0, r2 = 0;
    CHECK(ransacSphere(s, 0.01, 400, c1, r1, a, 7));
    ransacSphere(s, 0.01, 400, c2, r2, b, 7);
    CHECK(a == b && std::is_sorted(a.begin(), a.end()));
    CHECK_NEAR(r1, 0.7, 1e-3);
  }
#endif
  return TEST_SUMMARY();
}
