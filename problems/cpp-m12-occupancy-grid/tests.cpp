// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include "common/check.h"

template <class F>
static bool throwsOutOfRange(F f) {
  try { f(); } catch (const std::out_of_range&) { return true; } catch (...) { return false; }
  return false;
}

int main() {
  OccupancyGrid g(10, 5, 0.5, -2.0, -1.0);
  CHECK(g.width() == 10 && g.height() == 5);

  auto c = g.worldToCell(0.1, 0.2);  // (0.1+2)/0.5 = 4.2 -> 4, (0.2+1)/0.5 = 2.4 -> 2
  CHECK(c && c->x == 4 && c->y == 2);
  CHECK(!g.worldToCell(3.0, 0.0));  // beyond width (x range [-2, 3))

  g.update(4, 2, 1.5);
  g.update(4, 2, 10.0);
  CHECK_NEAR(g.logOdds(4, 2), 4.0, 1e-12);  // clamped
  CHECK(throwsOutOfRange([&] { g.update(10, 0, 1.0); }));

  bool threw = false;
  try { OccupancyGrid bad(0, 5, 0.5, 0, 0); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);

#ifdef HIDDEN
  // const access
  const OccupancyGrid& cg = g;
  CHECK_NEAR(cg.probability(4, 2), 1.0 - 1.0 / (1.0 + std::exp(4.0)), 1e-12);
  CHECK_NEAR(cg.probability(0, 0), 0.5, 1e-12);
  CHECK(throwsOutOfRange([&] { cg.logOdds(-1, 0); }) && throwsOutOfRange([&] { cg.probability(0, 5); }) &&
        throwsOutOfRange([&] { cg.cellCenter(10, 4); }));
  // negative coordinates use floor; boundaries are half-open
  auto n = g.worldToCell(-1.9, -0.9);
  CHECK(n && n->x == 0 && n->y == 0);
  CHECK(!g.worldToCell(-2.0001, 0.0) && !g.worldToCell(0.0, -1.0001) && !g.worldToCell(0.0, 1.5));
  auto edge = g.worldToCell(-2.0, -1.0);
  CHECK(edge && edge->x == 0 && edge->y == 0);
  auto last = g.worldToCell(2.999, 1.499);
  CHECK(last && last->x == 9 && last->y == 4);
  // cellCenter round-trips
  Vec2 ctr = g.cellCenter(9, 4);
  CHECK(std::abs(ctr.x - 2.75) < 1e-12 && std::abs(ctr.y - 1.25) < 1e-12);
  auto rt = g.worldToCell(ctr.x, ctr.y);
  CHECK(rt && rt->x == 9 && rt->y == 4);
  // lower clamp, row-major independence of cells
  g.update(0, 4, -100.0);
  CHECK_NEAR(g.logOdds(0, 4), -4.0, 1e-12);
  CHECK_NEAR(g.logOdds(4, 0), 0.0, 1e-12);
  CHECK_EQ(g.countOccupied(0.5), std::size_t(1));
  // factory
  CHECK(!OccupancyGrid::create(5, -1, 1.0, 0, 0) && !OccupancyGrid::create(5, 5, 0.0, 0, 0) &&
        !OccupancyGrid::create(5, 5, -0.1, 0, 0) && !OccupancyGrid::create(5, 5, std::nan(""), 0, 0));
  auto ok = OccupancyGrid::create(1000, 1000, 0.05, -25, -25);
  CHECK(ok && ok->width() == 1000 && ok->resolution() == 0.05);
  bool threwRes = false;
  try { OccupancyGrid bad(3, 3, -1.0, 0, 0); } catch (const std::invalid_argument&) { threwRes = true; }
  CHECK(threwRes);
  // big grid: copy is independent (rule of zero), many updates
  if (ok) {
    for (int y = 0; y < 1000; y += 10)
      for (int x = 0; x < 1000; x += 10) ok->update(x, y, 0.85);
    OccupancyGrid copy = *ok;
    copy.update(0, 0, -10);
    CHECK(ok->countOccupied(0.6) == 10000 && copy.countOccupied(0.6) == 9999);
    auto far = ok->worldToCell(24.99, -25.0);
    CHECK(far && far->x == 999 && far->y == 0);
  }
  CHECK(std::is_copy_constructible_v<OccupancyGrid> && std::is_nothrow_move_constructible_v<OccupancyGrid>);
#endif
  return TEST_SUMMARY();
}
