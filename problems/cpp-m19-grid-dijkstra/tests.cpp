// The candidate's code is injected before this file (g++ -include).
#include <random>
#include <unordered_set>
#include "common/check.h"

#ifdef HIDDEN
// Reference: O(V^2) Dijkstra on a dense array (different algorithmic shape from the expected solution).
static double refCost(const std::vector<std::vector<int>>& blk, int sx, int sy, int gx, int gy) {
  const int W = (int)blk.size(), H = (int)blk[0].size();
  std::vector<double> dist(W * H, 1e18);
  std::vector<char> done(W * H, 0);
  dist[sx * H + sy] = 0;
  for (int it = 0; it < W * H; ++it) {
    int u = -1;
    for (int i = 0; i < W * H; ++i) if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
    if (u < 0 || dist[u] >= 1e18) break;
    done[u] = 1;
    int ux = u / H, uy = u % H;
    for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
      if (!dx && !dy) continue;
      int vx = ux + dx, vy = uy + dy;
      if (vx < 0 || vy < 0 || vx >= W || vy >= H || blk[vx][vy]) continue;
      double nd = dist[u] + ((dx && dy) ? std::sqrt(2.0) : 1.0);
      if (nd < dist[vx * H + vy]) dist[vx * H + vy] = nd;
    }
  }
  return dist[gx * H + gy];
}
#endif

int main() {
  Cell c = toCell(-0.1, 0.25, 0.5);
  CHECK(c.x == -1 && c.y == 0);

  OccMap occ = buildOccupancy({{0.1, 0.1}, {0.2, 0.3}, {1.5, 0.1}, {-0.4, -0.4}}, 1.0);
  CHECK_EQ(occ.size(), 3u);
  const Cell c00{0, 0}, cmm{-1, -1};
  CHECK(occ.count(c00) && occ[c00] == 2);
  CHECK(occ.count(cmm) && occ[cmm] == 1);

  CellHash h;
  CHECK(h({1, 2}) != h({2, 1}));

  // open 10x10 box: pure diagonal
  OccMap empty;
  auto open = planCost(empty, 1, {0, 0}, {9, 9}, {0, 0}, {9, 9});
  CHECK(open.has_value());
  if (open) CHECK_NEAR(*open, 9 * std::sqrt(2.0), 1e-9);

  // wall at x=5 for y=0..8, gap at (5,9)
  OccMap wall;
  for (int y = 0; y <= 8; ++y) wall[{5, y}] = 3;
  auto around = planCost(wall, 1, {0, 0}, {9, 0}, {0, 0}, {9, 9});
  CHECK(around.has_value());
  if (around) CHECK_NEAR(*around, 9.0 + 9.0 * std::sqrt(2.0), 1e-9);

#ifdef HIDDEN
  // hash spreads a 64x64 neighbourhood (incl. negatives) and the diagonal
  std::unordered_set<std::size_t> hs;
  for (int x = -32; x < 32; ++x) for (int y = -32; y < 32; ++y) hs.insert(h({x, y}));
  CHECK(hs.size() >= 4000);
  std::unordered_set<std::size_t> diag;
  for (int k = 0; k < 100; ++k) diag.insert(h({k, k}));
  CHECK(diag.size() >= 95);
  // full wall -> unreachable; min_hits threshold makes the wall passable
  OccMap full = wall;
  full[{5, 9}] = 1;
  CHECK(!planCost(full, 1, {0, 0}, {9, 0}, {0, 0}, {9, 9}).has_value());
  auto thru = planCost(full, 2, {0, 0}, {9, 0}, {0, 0}, {9, 9});  // (5,9) has 1 hit < 2 -> free
  CHECK(thru && std::abs(*thru - (9.0 + 9.0 * std::sqrt(2.0))) < 1e-9);
  // blocked / out-of-box endpoints, start == goal
  CHECK(!planCost(wall, 1, {5, 0}, {9, 0}, {0, 0}, {9, 9}).has_value());
  CHECK(!planCost(wall, 1, {0, 0}, {10, 0}, {0, 0}, {9, 9}).has_value());
  auto zero = planCost(wall, 1, {3, 3}, {3, 3}, {0, 0}, {9, 9});
  CHECK(zero && *zero == 0.0);
  // negative-coordinate box, straight line
  auto neg = planCost(empty, 1, {-20, -5}, {-10, -5}, {-25, -6}, {-5, -4});
  CHECK(neg && std::abs(*neg - 10.0) < 1e-9);
  // const map must not be modified by the planner
  const OccMap cocc = wall;
  planCost(cocc, 1, {0, 0}, {9, 9}, {0, 0}, {9, 9});
  CHECK(cocc.size() == 9);
  // random 30x30 grids vs reference
  std::mt19937 rng(3);
  std::bernoulli_distribution obstacle(0.3);
  bool ok = true;
  for (int trial = 0; trial < 3 && ok; ++trial) {
    std::vector<std::vector<int>> blk(30, std::vector<int>(30, 0));
    OccMap o;
    for (int x = 0; x < 30; ++x) for (int y = 0; y < 30; ++y)
      if (obstacle(rng) && !(x == 0 && y == 0) && !(x == 29 && y == 29)) { blk[x][y] = 1; o[{x + 100, y - 50}] = 2; }
    double ref = refCost(blk, 0, 0, 29, 29);
    auto got = planCost(o, 2, {100, -50}, {129, -21}, {100, -50}, {129, -21});
    ok = (ref >= 1e18) ? !got.has_value() : (got.has_value() && std::abs(*got - ref) < 1e-9);
  }
  CHECK(ok);
  // bigger open box for speed
  auto far = planCost(empty, 1, {0, 0}, {199, 50}, {0, 0}, {199, 199});
  CHECK(far && std::abs(*far - (149.0 + 50 * std::sqrt(2.0))) < 1e-6);
#endif
  return TEST_SUMMARY();
}
