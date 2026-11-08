// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static bool samePt(const Point3& a, const Point3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

int main() {
  std::vector<Point3> c = {{3, 0, 0}, {1, 0, 0}, {0, 2, 0}, {0, 0, -0.5}};
  sortByDistance(c, {0, 0, 0});
  CHECK(samePt(c[0], {0, 0, -0.5}) && samePt(c[3], {3, 0, 0}));

  std::vector<Point3> r = {{0.1, 0, 0}, {5, 0, 0}, {0, 50, 0}, {0, 0, 2}};
  CHECK_EQ(removeOutOfRange(r, 0.5, 10.0), std::size_t(2));
  CHECK(r.size() == 2 && samePt(r[0], {5, 0, 0}) && samePt(r[1], {0, 0, 2}));

  auto box = makeBoxFilter({-1, -1, 0}, {1, 1, 2});
  CHECK(box({0, 0, 1}));
  CHECK(!box({0, 0, 3}));

  auto gen = makeIdGenerator(100);
  CHECK_EQ(gen(), 100);
  CHECK_EQ(gen(), 101);

#ifdef HIDDEN
  // stable: equal distances keep input order
  std::vector<Point3> tie = {{0, 2, 0}, {2, 0, 0}, {0, 0, 1}, {-2, 0, 0}};
  sortByDistance(tie, {0, 0, 0});
  CHECK(samePt(tie[1], {0, 2, 0}) && samePt(tie[2], {2, 0, 0}) && samePt(tie[3], {-2, 0, 0}));
  // non-origin reference point
  std::vector<Point3> o = {{0, 0, 0}, {10, 10, 10}};
  sortByDistance(o, {9, 9, 9});
  CHECK(samePt(o[0], {10, 10, 10}));
  // empty input & boundaries inclusive
  std::vector<Point3> e;
  sortByDistance(e, {1, 2, 3});
  CHECK_EQ(removeOutOfRange(e, 0, 1), std::size_t(0));
  std::vector<Point3> b = {{1, 0, 0}, {0, 2, 0}};
  CHECK_EQ(removeOutOfRange(b, 1.0, 2.0), std::size_t(0));
  // box: boundaries inclusive, negative coords; filter survives scope of its arguments
  auto make = [] { Point3 lo{-5, -5, -5}, hi{-1, -1, -1}; return makeBoxFilter(lo, hi); };
  auto neg = make();
  CHECK(neg({-1, -1, -1}) && neg({-5, -3, -2}) && !neg({0, -2, -2}));
  // generators are independent; copies carry state
  auto g = makeIdGenerator(-1);
  g(); g();
  auto h = g;
  CHECK_EQ(g(), 1);
  CHECK_EQ(h(), 1);
  auto other = makeIdGenerator(7);
  CHECK_EQ(other(), 7);
  // larger cloud: sorted check
  std::vector<Point3> big;
  for (int i = 0; i < 20000; ++i) big.push_back({double((i * 7919) % 1000), double(i % 13), 0.0});
  sortByDistance(big, {0, 0, 0});
  bool sorted = true;
  for (std::size_t i = 1; i < big.size(); ++i) {
    const double a = big[i - 1].x * big[i - 1].x + big[i - 1].y * big[i - 1].y;
    const double bb = big[i].x * big[i].x + big[i].y * big[i].y;
    if (a > bb) sorted = false;
  }
  CHECK(sorted);
#endif
  return TEST_SUMMARY();
}
