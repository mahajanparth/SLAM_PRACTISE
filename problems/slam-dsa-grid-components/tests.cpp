// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

using Grid = std::vector<std::vector<int>>;

int main() {
  Grid g = {
      {0, 0, 1, 0},
      {0, 1, 1, 0},
      {1, 1, 0, 0},
      {0, 1, 0, 1},
  };
  CHECK_EQ(countFreeRegions(g), 3);
  Grid L = labelFreeRegions(g);
  CHECK(L.size() == 4 && L[0].size() == 4);
  if (L.size() == 4 && L[0].size() == 4) {
    CHECK_EQ(L[0][0], 0);
    CHECK_EQ(L[1][0], 0);
    CHECK_EQ(L[0][3], 1);
    CHECK_EQ(L[3][2], 1);  // connected via (2,2),(2,3),(1,3)
    CHECK_EQ(L[3][0], 2);
    CHECK_EQ(L[0][2], -1);
  }

#ifdef HIDDEN
  CHECK_EQ(countFreeRegions({}), 0);
  CHECK_EQ(countFreeRegions({{1, 1}, {1, 1}}), 0);
  CHECK_EQ(countFreeRegions({{0}}), 1);
  // diagonal neighbours do NOT connect
  CHECK_EQ(countFreeRegions({{0, 1}, {1, 0}}), 2);
  // checkerboard 6x6: 18 isolated free cells
  Grid cb(6, std::vector<int>(6));
  for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) cb[r][c] = (r + c) % 2;
  CHECK_EQ(countFreeRegions(cb), 18);
  // single row / column
  CHECK_EQ(countFreeRegions({{0, 0, 1, 0, 1, 1, 0}}), 3);
  CHECK_EQ(countFreeRegions({{0}, {1}, {0}, {0}}), 2);
  // large open map with a wall: must not blow the stack
  const int N = 600;
  Grid big(N, std::vector<int>(N, 0));
  for (int r = 0; r < N; ++r) big[r][300] = 1;
  CHECK_EQ(countFreeRegions(big), 2);
  // spiral-ish snake corridor: one region, labels row-major
  Grid s(9, std::vector<int>(9, 1));
  for (int c = 0; c < 9; ++c) { s[0][c] = 0; s[4][c] = 0; s[8][c] = 0; }
  for (int r = 0; r < 4; ++r) s[r][8] = 0;
  for (int r = 4; r < 9; ++r) s[r][0] = 0;
  s[2][2] = 0;  // isolated pocket -> second region in scan order
  Grid sl = labelFreeRegions(s);
  CHECK_EQ(countFreeRegions(s), 2);
  CHECK(sl.size() == 9 && sl[8][8] == 0 && sl[2][2] == 1);
#endif
  return TEST_SUMMARY();
}
