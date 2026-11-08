// The candidate's code is injected before this file (g++ -include).
#include <memory>
#include <string>
#include "common/check.h"

struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
};

int main() {
  RingBuffer<int, 3> rb;
  CHECK(rb.empty());
  rb.push(1); rb.push(2);
  CHECK_EQ(rb.size(), 2u);
  CHECK_EQ(rb.front(), 1);
  rb.push(3); rb.push(4);  // overwrites 1
  CHECK(rb.full());
  CHECK_EQ(rb.size(), 3u);
  if (rb.size() == 3) { CHECK_EQ(rb[0], 2); CHECK_EQ(rb[2], 4); CHECK_EQ(rb.back(), 4); }
  auto p = rb.pop();
  CHECK(p.has_value() && *p == 2);
  CHECK_EQ(rb.size(), 2u);

  RingBuffer<double, 4> gyro;
  for (double g : {1.0, 2.0, 3.0, 4.0, 5.0}) gyro.push(g);  // keeps 2,3,4,5
  CHECK_NEAR(average(gyro), 3.5, 1e-12);
  static_assert(RingBuffer<double, 4>::capacity() == 4, "capacity");

#ifdef HIDDEN
  // pop until empty, then nullopt
  RingBuffer<int, 2> s;
  CHECK(!s.pop().has_value());
  s.push(7);
  CHECK(s.pop().value_or(-1) == 7);
  CHECK(!s.pop().has_value());
  CHECK(s.empty());
  // wrap-around many times
  RingBuffer<int, 5> w;
  for (int i = 0; i < 1003; ++i) w.push(i);
  CHECK(w.size() == 5 && w.front() == 998 && w.back() == 1002 && w[3] == 1001);
  // interleaved push/pop across the wrap
  RingBuffer<int, 3> q;
  bool ok = true;
  int next_out = 0;
  for (int i = 0; i < 50; ++i) {
    q.push(i);
    if (i % 2 == 1) { auto v = q.pop(); ok = ok && v && *v == next_out; ++next_out; }
    while (q.size() == 3) { auto v = q.pop(); ok = ok && v && *v == next_out; ++next_out; }
  }
  CHECK(ok);
  // N == 1
  RingBuffer<std::string, 1> one;
  one.push(std::string("a")); one.push(std::string("b"));
  CHECK(one.size() == 1 && one.front() == "b" && one.back() == "b");
  // move-only type
  RingBuffer<std::unique_ptr<int>, 2> up;
  up.push(std::make_unique<int>(1));
  up.push(std::make_unique<int>(2));
  up.push(std::make_unique<int>(3));  // drops 1 (no leak)
  auto u = up.pop();
  CHECK(u && *u && **u == 2);
  CHECK(up.size() == 1 && *up.front() == 3);
  // clear then reuse
  w.clear();
  CHECK(w.empty() && !w.full());
  w.push(42);
  CHECK(w.size() == 1 && w[0] == 42 && w.front() == 42);
  // average: empty and struct type; modify through operator[]
  RingBuffer<Vec3, 3> acc;
  CHECK_NEAR(average(acc).x, 0.0, 1e-15);
  acc.push({0, 0, 9.81}); acc.push({1, 0, 9.81}); acc.push({2, 3, 9.81}); acc.push({3, 3, 9.81});
  Vec3 m = average(acc);
  CHECK_NEAR(m.x, 2.0, 1e-12); CHECK_NEAR(m.y, 2.0, 1e-12); CHECK_NEAR(m.z, 9.81, 1e-12);
  acc[0].x = 100;
  CHECK_NEAR(acc.front().x, 100.0, 1e-12);
#endif
  return TEST_SUMMARY();
}
