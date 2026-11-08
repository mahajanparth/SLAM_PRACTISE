// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static std::vector<Point3> makeCloud(int n) {
  std::vector<Point3> c;
  for (int i = 0; i < n; ++i) c.push_back({double(i), 1.0, -double(i)});
  return c;
}

int main() {
  const double PI = std::acos(-1.0);
  Pose T = makePose(PI / 2, 1, 2, 3);
  CHECK_NEAR(T.yaw, PI / 2, 1e-12);
  CHECK_NEAR(T.tz, 3.0, 1e-12);
  Pose R = makePose(PI / 2);  // defaults -> zero translation
  CHECK_NEAR(R.tx + R.ty + R.tz, 0.0, 1e-12);

  Point3 p = apply(T, Point3{1, 0, 0});  // rotate x-axis onto y, then translate
  CHECK_NEAR(p.x, 1.0, 1e-9);
  CHECK_NEAR(p.y, 3.0, 1e-9);
  CHECK_NEAR(p.z, 3.0, 1e-9);

  const std::vector<Point3> src = makeCloud(4);
  std::vector<Point3> out = apply(T, src);  // lvalue -> copy overload
  CHECK(out.size() == 4);
  CHECK_NEAR(src[2].x, 2.0, 1e-12);  // source untouched
  if (out.size() == 4) CHECK_NEAR(out[2].y, 2.0 + 2.0, 1e-9);

  std::vector<Point3> owned = makeCloud(5);
  const Point3* buf = owned.data();
  std::vector<Point3> moved = apply(T, std::move(owned));  // rvalue -> in-place overload
  CHECK(moved.size() == 5);
  CHECK(moved.data() == buf);  // same buffer, no allocation

#ifdef HIDDEN
  // in place
  std::vector<Point3> c = makeCloud(3);
  applyInPlace(makePose(PI), c);
  CHECK_NEAR(c[1].x, -1.0, 1e-9);
  CHECK_NEAR(c[1].y, -1.0, 1e-9);
  CHECK_NEAR(c[1].z, -1.0, 1e-9);
  // empty clouds through every overload
  std::vector<Point3> e;
  applyInPlace(T, e);
  CHECK(e.empty());
  CHECK(apply(T, e).empty());
  CHECK(apply(T, std::vector<Point3>{}).empty());
  // the copy overload allocates exactly once
  std::vector<Point3> big = makeCloud(1000);
  std::vector<Point3> bigOut = apply(T, big);
  CHECK(bigOut.capacity() == 1000);
  // identity pose, negative yaw wrap
  Point3 q = apply(makePose(0), Point3{-4, 5, -6});
  CHECK_NEAR(q.x, -4.0, 1e-12);
  CHECK_NEAR(q.z, -6.0, 1e-12);
  Point3 w = apply(makePose(-PI / 2), Point3{0, 1, 0});
  CHECK_NEAR(w.x, 1.0, 1e-9);
  CHECK_NEAR(w.y, 0.0, 1e-9);
  // rvalue overload produces correct values, not just the right buffer
  std::vector<Point3> r = apply(T, makeCloud(3));
  CHECK(r.size() == 3);
  if (r.size() == 3) { CHECK_NEAR(r[2].x, 1.0 - 1.0, 1e-9); CHECK_NEAR(r[2].z, -2.0 + 3.0, 1e-9); }
#endif
  return TEST_SUMMARY();
}
