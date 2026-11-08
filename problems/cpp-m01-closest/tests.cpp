// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  CHECK_NEAR(range({3, 4, 0}), 5.0, 1e-12);
  CHECK_NEAR(range({0, 0, 0}), 0.0, 1e-12);

  std::vector<Point3> c = {{1, 0, 0}, {0, 2, 0}, {0, 0, 3}};
  scaleInPlace(c, 2.0);
  CHECK_NEAR(c[1].y, 4.0, 1e-12);
  CHECK_NEAR(c[2].z, 6.0, 1e-12);

  const Point3* best = findClosest(c, {0, 3.9, 0});
  CHECK(best != nullptr);
  CHECK(best == &c[1]);  // must point INTO the cloud
  CHECK(findClosest({}, {0, 0, 0}) == nullptr);

  std::vector<Point3> line = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  CHECK_NEAR(pathLength(line), 2.0, 1e-12);
  CHECK_NEAR(pathLength({}), 0.0, 1e-12);

#ifdef HIDDEN
  CHECK_NEAR(pathLength({{5, 5, 5}}), 0.0, 1e-12);
  CHECK_NEAR(range({-3, -4, 12}), 13.0, 1e-12);
  std::vector<Point3> neg = {{-1, -1, -1}};
  scaleInPlace(neg, -2.0);
  CHECK_NEAR(neg[0].x, 2.0, 1e-12);
  std::vector<Point3> empty;
  scaleInPlace(empty, 3.0);
  CHECK(empty.empty());
  // ties -> first occurrence; large cloud
  std::vector<Point3> big(200000, Point3{1, 1, 1});
  big[123456] = {0, 0, 0.5};
  CHECK(findClosest(big, {0, 0, 0}) == &big[123456]);
  std::vector<Point3> tie = {{1, 0, 0}, {-1, 0, 0}};
  CHECK(findClosest(tie, {0, 0, 0}) == &tie[0]);
#endif
  return TEST_SUMMARY();
}
