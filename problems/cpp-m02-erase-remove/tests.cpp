// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static std::vector<Point3> line(int n) {  // points (i, 0, 0)
  std::vector<Point3> c;
  for (int i = 0; i < n; ++i) c.push_back({double(i), 0, 0});
  return c;
}

int main() {
  std::vector<Point3> c = {{0.1, 0, 0}, {3, 4, 0}, {0, 0, 50}, {1, 0, 0}, {0, -2, 0}};
  removeOutOfRange(c, 0.5, 10.0);
  CHECK(c.size() == 3);
  if (c.size() == 3) { CHECK_NEAR(c[0].x, 3.0, 1e-12); CHECK_NEAR(c[2].y, -2.0, 1e-12); }  // order kept

  std::vector<Point3> l = line(7);  // remove indices 2 and 5
  removeEveryNth(l, 3);
  CHECK(l.size() == 5);
  if (l.size() == 5) { CHECK_NEAR(l[2].x, 3.0, 1e-12); CHECK_NEAR(l[4].x, 6.0, 1e-12); }

  std::vector<Point3> d = {{0, 0, 0}, {0, 0, 0.001}, {1, 1, 1}, {1, 1, 1}, {0, 0, 0}};
  CHECK_EQ(dedupConsecutive(d, 0.01), std::size_t(2));
  CHECK(d.size() == 3);

#ifdef HIDDEN
  // inclusive bounds; everything removed; empty input
  std::vector<Point3> b = {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  removeOutOfRange(b, 1.0, 2.0);
  CHECK(b.size() == 2);
  removeOutOfRange(b, 100, 200);
  CHECK(b.empty());
  removeOutOfRange(b, 0, 1);
  CHECK(b.empty());
  // n == 0 no-op, n == 1 empties, n > size no-op
  std::vector<Point3> l2 = line(4);
  removeEveryNth(l2, 0);
  CHECK(l2.size() == 4);
  removeEveryNth(l2, 10);
  CHECK(l2.size() == 4);
  removeEveryNth(l2, 1);
  CHECK(l2.empty());
  // large: n = 2 keeps evens, O(n)
  std::vector<Point3> big = line(200000);
  removeEveryNth(big, 2);
  CHECK(big.size() == 100000);
  if (big.size() == 100000) CHECK_NEAR(big[99999].x, 199998.0, 1e-9);
  // large range filter with negative coordinates
  std::vector<Point3> neg = line(200000);
  for (auto& p : neg) p.x = -p.x;
  removeOutOfRange(neg, 10.0, 1000.0);
  CHECK(neg.size() == 991);
  if (!neg.empty()) CHECK_NEAR(neg.front().x, -10.0, 1e-12);
  // dedup: drift relative to LAST KEPT point; empty; single
  std::vector<Point3> drift = {{0, 0, 0}, {0.006, 0, 0}, {0.012, 0, 0}};
  CHECK_EQ(dedupConsecutive(drift, 0.01), std::size_t(1));
  CHECK(drift.size() == 2);
  std::vector<Point3> none;
  CHECK_EQ(dedupConsecutive(none, 1.0), std::size_t(0));
  std::vector<Point3> one = {{5, 5, 5}};
  CHECK_EQ(dedupConsecutive(one, 1.0), std::size_t(0));
#endif
  return TEST_SUMMARY();
}
