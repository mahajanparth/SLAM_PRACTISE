// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include <vector>
#include "common/check.h"

static void resetCounters() { PointBuffer::copies = 0; PointBuffer::moves = 0; }

int main() {
  PointBuffer a(3, {1, 2, 3});
  CHECK_EQ(a.size(), std::size_t(3));

  resetCounters();
  PointBuffer b = a;
  CHECK(b.size() == 3 && b.data() != a.data() && PointBuffer::copies == 1);
  if (b.size() == 3) b[0].x = 42;
  CHECK(a.size() == 3 && a[0].x == 1);  // deep copy

  const Point3* buf = a.data();
  PointBuffer c = std::move(a);
  CHECK(c.data() == buf && c.size() == 3 && PointBuffer::moves == 1);
  CHECK(a.size() == 0 && a.data() == nullptr);

  resetCounters();
  std::vector<PointBuffer> v;
  for (int i = 0; i < 20; ++i) v.emplace_back(4, Point3{double(i), 0, 0});
  CHECK(PointBuffer::copies == 0 && PointBuffer::moves > 0);

#ifdef HIDDEN
  CHECK(std::is_nothrow_move_constructible_v<PointBuffer> && std::is_nothrow_move_assignable_v<PointBuffer>);
  CHECK(v.size() == 20 && v[19].size() == 4 && v[19][3].x == 19);
  // empty
  PointBuffer e(0);
  CHECK(e.size() == 0 && e.data() == nullptr);
  PointBuffer e2 = e;
  CHECK(e2.size() == 0 && e2.data() == nullptr);
  // copy assignment: different sizes, self, from empty
  resetCounters();
  PointBuffer x(2, {1, 1, 1}), y(5, {2, 2, 2});
  x = y;
  CHECK(x.size() == 5 && x.data() != y.data() && x[4].x == 2 && PointBuffer::copies == 1);
  PointBuffer& xr = x;
  x = xr;
  CHECK(x.size() == 5 && x[0].x == 2 && PointBuffer::copies == 2);
  x = e;
  CHECK(x.size() == 0);
  // move assignment: steals, frees old, self-move no-op
  resetCounters();
  PointBuffer m(10, {7, 7, 7});
  const Point3* yb = y.data();
  m = std::move(y);
  CHECK(m.data() == yb && m.size() == 5 && y.size() == 0 && y.data() == nullptr && PointBuffer::moves == 1);
  PointBuffer& mr = m;
  m = std::move(mr);
  CHECK(m.size() == 5 && m.data() == yb && m[2].z == 2);
  // moved-from object is reusable
  y = PointBuffer(1, {9, 9, 9});
  CHECK(y.size() == 1 && y[0].y == 9);
  // vector copy = deep copies; erase in the middle shifts by move assignment
  resetCounters();
  std::vector<PointBuffer> w = v;
  CHECK(PointBuffer::copies == 20 && w[5].data() != v[5].data());
  resetCounters();
  w.erase(w.begin());
  CHECK(PointBuffer::copies == 0 && PointBuffer::moves == 19 && w[0][0].x == 1);
#endif
  return TEST_SUMMARY();
}
