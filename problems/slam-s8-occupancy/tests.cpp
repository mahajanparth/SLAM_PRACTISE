// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <limits>

using Cells = std::vector<std::pair<int, int>>;

int main() {
  CHECK(bresenham(0, 0, 5, 2) == Cells({{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}}));
  CHECK(bresenham(3, 3, 3, 3) == Cells({{3, 3}}));

  OccupancyGrid g(20, 20, 0.5, Eigen::Vector2d(-5, -5));
  int cx = -99, cy = -99;
  CHECK(g.worldToCell({0.25, 0.25}, cx, cy));
  CHECK(cx == 10 && cy == 10);
  CHECK(!g.worldToCell({-5.1, 0.0}, cx, cy));
  CHECK(cx == -1);
  CHECK_NEAR(g.probability(0, 0), 0.5, 1e-12);

  // single beam along +x, hit at 2 m -> cells 10..13 free, 14 occupied
  g.integrateScan({0.25, 0.25, 0.0}, {2.0}, 0.0, 0.0, 10.0);
  bool freeOk = true;
  for (int x = 10; x <= 13; ++x) freeOk = freeOk && std::abs(g.logOdds(x, 10) - OccupancyGrid::kLogFree) < 1e-12;
  CHECK(freeOk);
  CHECK_NEAR(g.logOdds(14, 10), OccupancyGrid::kLogOcc, 1e-12);
  CHECK_NEAR(g.logOdds(15, 10), 0.0, 1e-12);
  CHECK(g.probability(14, 10) > 0.65 && g.probability(12, 10) < 0.45);

#ifdef HIDDEN
  // steep, negative, reversed slopes
  CHECK(bresenham(0, 0, -2, -5) == Cells({{0, 0}, {0, -1}, {-1, -2}, {-1, -3}, {-2, -4}, {-2, -5}}));
  CHECK(bresenham(5, 2, 0, 0) == Cells({{5, 2}, {4, 2}, {3, 1}, {2, 1}, {1, 0}, {0, 0}}));
  CHECK(bresenham(0, 0, 0, -3) == Cells({{0, 0}, {0, -1}, {0, -2}, {0, -3}}));
  CHECK(bresenham(1, 1, -3, -3) == Cells({{1, 1}, {0, 0}, {-1, -1}, {-2, -2}, {-3, -3}}));
  {
    auto c = bresenham(2, -7, -9, 4);  // diagonal-ish negative slope: endpoints + 8-adjacency
    bool adj = !c.empty() && c.front() == std::make_pair(2, -7) && c.back() == std::make_pair(-9, 4);
    for (size_t i = 1; i < c.size(); ++i)
      adj = adj && std::abs(c[i].first - c[i - 1].first) <= 1 && std::abs(c[i].second - c[i - 1].second) <= 1;
    CHECK(adj);
    CHECK_EQ(c.size(), size_t(12));
  }

  // max-range beam (no hit) pointing -y: all free, nothing occupied
  OccupancyGrid m(20, 20, 0.5, Eigen::Vector2d(-5, -5));
  m.integrateScan({0.25, 0.25, M_PI / 2}, {3.0, std::numeric_limits<double>::infinity(), -1.0},
                  M_PI, 0.3, 3.0);  // first beam at angle 3pi/2 (down) with r == max_range
  CHECK_NEAR(m.logOdds(10, 4), OccupancyGrid::kLogFree, 1e-12);  // y = 0.25-3 = -2.75 -> cell 4
  CHECK_NEAR(m.logOdds(10, 7), OccupancyGrid::kLogFree, 1e-12);
  CHECK_NEAR(m.logOdds(10, 3), 0.0, 1e-12);

  // beam leaving the map (and robot near the edge): no crash, in-map cells free
  OccupancyGrid o(20, 20, 0.5, Eigen::Vector2d(-5, -5));
  o.integrateScan({4.75, 0.25, 0.0}, {50.0}, 0.0, 0.0, 100.0);
  CHECK_NEAR(o.logOdds(19, 10), OccupancyGrid::kLogFree, 1e-12);
  // robot outside the map, beam entering it: hit cell inside gets occupied
  o.integrateScan({-7.25, -4.75, 0.0}, {5.0}, 0.0, 0.0, 100.0);  // end x=-2.25 -> cell 5, row 0
  CHECK_NEAR(o.logOdds(5, 0), OccupancyGrid::kLogOcc, 1e-12);
  CHECK_NEAR(o.logOdds(0, 0), OccupancyGrid::kLogFree, 1e-12);

  // clamping after repeated scans; diagonal beam at 225 degrees (negative slope in both axes)
  OccupancyGrid k(20, 20, 0.5, Eigen::Vector2d(-5, -5));
  for (int i = 0; i < 20; ++i) k.integrateScan({0.25, 0.25, 0.0}, {2.0 * std::sqrt(2.0)}, -3 * M_PI / 4, 0.0, 10.0);
  CHECK_NEAR(k.logOdds(6, 6), OccupancyGrid::kLogMax, 1e-12);
  CHECK_NEAR(k.logOdds(8, 8), OccupancyGrid::kLogMin, 1e-12);
  CHECK(k.probability(6, 6) > 0.95);
  CHECK(k.worldToCell({-0.01, -0.01}, cx, cy) && cx == 9 && cy == 9);
#endif
  return TEST_SUMMARY();
}
