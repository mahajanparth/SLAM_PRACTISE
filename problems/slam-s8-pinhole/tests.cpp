// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  const Intrinsics pin{500, 500, 320, 240, 0, 0, 0, 0};
  const Intrinsics dist{500, 520, 320, 240, -0.2, 0.05, 0.001, -0.002};
  Eigen::Vector2d uv(-1, -1);

  CHECK(projectPoint(pin, {0, 0, 2}, uv));
  CHECK_MAT_NEAR(uv, Eigen::Vector2d(320, 240), 1e-12);
  CHECK(projectPoint(pin, {1, -0.5, 4}, uv));
  CHECK_MAT_NEAR(uv, Eigen::Vector2d(445, 177.5), 1e-12);

  // distorted point, hand-computed
  CHECK(projectPoint(dist, {0.4, 0.2, 2}, uv));
  {
    double x = 0.2, y = 0.1, r2 = 0.05, rad = 1 - 0.2 * r2 + 0.05 * r2 * r2;
    double xd = x * rad + 2 * 0.001 * x * y - 0.002 * (r2 + 2 * x * x);
    double yd = y * rad + 0.001 * (r2 + 2 * y * y) + 2 * -0.002 * x * y;
    CHECK_MAT_NEAR(uv, Eigen::Vector2d(500 * xd + 320, 520 * yd + 240), 1e-9);
  }

  // behind camera
  uv << 7, 7;
  CHECK(!projectPoint(pin, {0, 0, -1}, uv));
  CHECK_MAT_NEAR(uv, Eigen::Vector2d(7, 7), 0.0);

  std::vector<Eigen::Vector3d> cloud = {{0, 0, 1}, {0, 0, -1}, {10, 0, 1}, {0.1, 0.1, 1}};
  std::vector<Eigen::Vector2d> px;
  auto idx = projectToImage(pin, cloud, 640, 480, px);
  CHECK(idx == std::vector<int>({0, 3}));
  CHECK_EQ(px.size(), idx.size());

#ifdef HIDDEN
  // Z exactly 0, tiny positive Z, NaN
  CHECK(!projectPoint(pin, {1, 1, 0}, uv));
  CHECK(!projectPoint(pin, {1, 1, 1e-9}, uv));
  CHECK(!projectPoint(pin, {0, 0, std::nan("")}, uv));
  // behind camera would project INTO the image if not rejected (mirror)
  std::vector<Eigen::Vector3d> mirror = {{-0.1, -0.1, -1}};
  CHECK(projectToImage(pin, mirror, 640, 480, px).empty());
  CHECK(px.empty());
  // image bounds: u == width is outside, u == 0 inside
  std::vector<Eigen::Vector3d> edge = {{-0.64, 0, 1}, {0.64, 0, 1}, {0, 0.479, 1}};
  auto eidx = projectToImage(pin, edge, 640, 480, px);
  CHECK(eidx == std::vector<int>({0, 2}));
  if (px.size() == 2) CHECK_NEAR(px[0].x(), 0.0, 1e-9);
  // empty cloud, stale output cleared
  px.assign(3, Eigen::Vector2d::Zero());
  CHECK(projectToImage(pin, {}, 640, 480, px).empty());
  CHECK(px.empty());
  // pure tangential, on-axis point: r = 0 -> stays at principal point
  Intrinsics tang{400, 400, 100, 50, 0.3, 0.1, 0.01, 0.02};
  CHECK(projectPoint(tang, {0, 0, 5}, uv));
  CHECK_MAT_NEAR(uv, Eigen::Vector2d(100, 50), 1e-12);
#endif
  return TEST_SUMMARY();
}
