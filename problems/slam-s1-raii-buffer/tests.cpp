// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include <vector>
#include "common/check.h"

static bool allEqual(const FloatBuffer& b, float v) {
  for (std::size_t i = 0; i < b.size(); ++i)
    if (b[i] != v) return false;
  return true;
}

int main() {
  FloatBuffer a(5);
  CHECK(a.size() == 5);
  CHECK(a.data() != nullptr);
  if (a.size() == 5 && a.data()) {
    CHECK(allEqual(a, 0.0f));  // zero-initialised
    for (std::size_t i = 0; i < 5; ++i) a[i] = float(i);
  }

  FloatBuffer b(a);  // deep copy
  CHECK(b.size() == 5);
  CHECK(b.data() != a.data());
  if (b.size() == 5 && a.size() == 5 && b.data() && b.data() != a.data()) {
    b[0] = 42.0f;
    CHECK_NEAR(a[0], 0.0, 0.0);
    CHECK_NEAR(b[4], 4.0, 0.0);
  }

  FloatBuffer c(std::move(a));  // move: steal
  CHECK(c.size() == 5);
  CHECK(a.size() == 0 && a.data() == nullptr);
  if (c.size() == 5 && c.data()) CHECK_NEAR(c[3], 3.0, 0.0);

#ifdef HIDDEN
  // copy assignment into a non-empty buffer of a different size
  FloatBuffer d(2);
  d = c;
  CHECK(d.size() == 5);
  if (d.size() == 5 && d.data() && d.data() != c.data()) CHECK_NEAR(d[4], 4.0, 0.0);
  // self copy-assignment
  FloatBuffer& dref = d;
  d = dref;
  CHECK(d.size() == 5);
  if (d.size() == 5 && d.data()) CHECK_NEAR(d[2], 2.0, 0.0);
  // self move-assignment must not destroy the data
  d = std::move(dref);
  CHECK(d.size() == 5);
  // move assignment
  FloatBuffer e(3);
  e = std::move(d);
  CHECK(e.size() == 5 && d.size() == 0 && d.data() == nullptr);
  // moved-from object can be reused
  d = FloatBuffer(7);
  CHECK(d.size() == 7);
  // copy of an empty buffer
  FloatBuffer empty;
  FloatBuffer ecopy(empty);
  CHECK(ecopy.size() == 0 && ecopy.data() == nullptr);
  FloatBuffer zero(0);
  CHECK(zero.size() == 0);
  // const access
  const FloatBuffer& ce = e;
  if (ce.size() == 5 && ce.data()) CHECK_NEAR(ce[1], 1.0, 0.0);
  // noexcept moves -> std::vector moves on reallocation
  CHECK(std::is_nothrow_move_constructible<FloatBuffer>::value);
  CHECK(std::is_nothrow_move_assignable<FloatBuffer>::value);
  std::vector<FloatBuffer> v;
  for (int i = 0; i < 20; ++i) v.emplace_back(100);
  const float* first = v[0].data();
  for (int i = 0; i < 100; ++i) v.emplace_back(10);  // several reallocations
  CHECK(v[0].data() == first);  // moved, not copied
  // a large buffer copied many times: no leaks
  FloatBuffer big(100000);
  for (int i = 0; i < 20; ++i) { FloatBuffer t(big); t = big; }
  CHECK(big.size() == 100000);
#endif
  return TEST_SUMMARY();
}
