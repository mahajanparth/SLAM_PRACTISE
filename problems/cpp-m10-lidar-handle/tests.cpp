// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include "common/check.h"

int main() {
  {
    LidarHandle h(3);
    CHECK(h.valid());
    CHECK_EQ(g_open_handles, 1);
    auto r = h.read(4);
    CHECK(r.size() == 4 && r[0] == 300.0f && r[3] == 300.75f);
    LidarHandle m(std::move(h));
    CHECK(!h.valid() && m.valid());
    CHECK_EQ(g_open_handles, 1);
  }
  CHECK_EQ(g_open_handles, 0);

  bool threw = false;
  try { LidarHandle bad(-1); } catch (const std::runtime_error&) { threw = true; }
  CHECK(threw);

#ifdef HIDDEN
  CHECK(std::is_nothrow_move_constructible_v<LidarHandle> && !std::is_copy_constructible_v<LidarHandle>);
  // move-assign closes the old target device
  {
    LidarHandle a(1), b(2);
    CHECK_EQ(g_open_handles, 2);
    a = std::move(b);
    CHECK_EQ(g_open_handles, 1);
    CHECK(a.valid() && a.get() && a.get()->port == 2 && !b.valid());
    LidarHandle& self = a;
    a = std::move(self);  // self-move: no-op
    CHECK(a.valid() && g_open_handles == 1);
  }
  CHECK_EQ(g_open_handles, 0);
  // release / reset
  {
    LidarHandle a(5);
    c_lidar* raw = a.release();
    CHECK(!a.valid() && raw && g_open_handles == 1);
    LidarHandle b;
    b.reset(raw);
    b.reset(b.get());  // must not close
    CHECK(b.valid() && g_open_handles == 1);
    b.reset();
    CHECK(!b.valid() && g_open_handles == 0);
  }
  // read edge cases
  {
    LidarHandle e;
    bool logic = false;
    try { e.read(3); } catch (const std::logic_error&) { logic = true; }
    CHECK(logic);
    LidarHandle h(7);
    CHECK(h.read(0).empty());
    auto second = (h.read(1), h.read(1));
    CHECK(second.size() == 1 && second[0] == 701.0f);
  }
  // openAll: success and all-or-nothing
  {
    auto hs = openAll({1, 2, 3, 4});
    CHECK(hs.size() == 4 && g_open_handles == 4 && hs[2].valid() && hs[2].get()->port == 3);
  }
  CHECK_EQ(g_open_handles, 0);
  bool threwAll = false;
  try { auto hs = openAll({1, 2, -9, 4}); } catch (const std::runtime_error&) { threwAll = true; }
  CHECK(threwAll && g_open_handles == 0);
  // vector growth with move-only handles
  {
    std::vector<LidarHandle> v;
    for (int i = 0; i < 33; ++i) v.emplace_back(i);
    CHECK(g_open_handles == 33 && v[32].get() && v[32].get()->port == 32);
  }
  CHECK_EQ(g_open_handles, 0);
#endif
  return TEST_SUMMARY();
}
