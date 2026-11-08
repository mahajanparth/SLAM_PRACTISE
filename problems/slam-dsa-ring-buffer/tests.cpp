// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <string>

template <typename F>
bool throwsOutOfRange(F f) {
  try { f(); } catch (const std::out_of_range&) { return true; } catch (...) { return false; }
  return false;
}

int main() {
  RingBuffer<int, 3> rb;
  CHECK(rb.empty());
  rb.push(1); rb.push(2);
  CHECK_EQ(rb.size(), std::size_t(2));
  CHECK_EQ(rb[0], 1);
  CHECK_EQ(rb.back(), 2);
  rb.push(3); rb.push(4);  // overwrites 1
  CHECK(rb.full());
  CHECK_EQ(rb.size(), std::size_t(3));
  CHECK_EQ(rb[0], 2);
  CHECK_EQ(rb[2], 4);
  CHECK_EQ(rb.back(), 4);

#ifdef HIDDEN
  // wrap around many times
  RingBuffer<int, 5> w;
  for (int k = 0; k < 1003; ++k) w.push(k);
  CHECK_EQ(w.size(), std::size_t(5));
  bool ok = true;
  for (std::size_t i = 0; i < 5; ++i) ok = ok && (w[i] == 998 + int(i));
  CHECK(ok);
  CHECK_EQ(w.back(), 1002);
  // N = 1
  RingBuffer<std::string, 1> one;
  one.push("a"); one.push("b");
  CHECK_EQ(one.size(), std::size_t(1));
  CHECK_EQ(one[0], std::string("b"));
  CHECK_EQ(one.back(), std::string("b"));
  // access on empty / out of range
  RingBuffer<double, 4> e;
  CHECK(throwsOutOfRange([&] { (void)e.back(); }));
  CHECK(throwsOutOfRange([&] { (void)e[0]; }));
  e.push(1.5);
  CHECK(throwsOutOfRange([&] { (void)e[1]; }));
  // clear resets
  w.clear();
  CHECK(w.empty());
  w.push(7);
  CHECK_EQ(w[0], 7);
  CHECK_EQ(w.back(), 7);
  static_assert(RingBuffer<int, 7>::capacity() == 7, "capacity");
#endif
  return TEST_SUMMARY();
}
