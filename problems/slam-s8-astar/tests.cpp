// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <random>

// Reference Dijkstra (same move rules) -> optimal cost or -1.
static double dijkstra(const Grid& g, Cell s, Cell t) {
  if (!g.free(s.first, s.second) || !g.free(t.first, t.second)) return -1;
  std::vector<double> d(g.rows * g.cols, 1e18);
  using QE = std::pair<double, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<>> q;
  d[s.first * g.cols + s.second] = 0; q.push({0, s.first * g.cols + s.second});
  while (!q.empty()) {
    auto [du, u] = q.top(); q.pop();
    if (du > d[u]) continue;
    int r = u / g.cols, c = u % g.cols;
    for (int dr = -1; dr <= 1; ++dr) for (int dc = -1; dc <= 1; ++dc) {
      if (!dr && !dc) continue;
      if (!g.free(r + dr, c + dc)) continue;
      if (dr && dc && (!g.free(r + dr, c) || !g.free(r, c + dc))) continue;
      double nd = du + ((dr && dc) ? std::sqrt(2.0) : 1.0);
      int v = (r + dr) * g.cols + c + dc;
      if (nd < d[v]) { d[v] = nd; q.push({nd, v}); }
    }
  }
  double r = d[t.first * g.cols + t.second];
  return r > 1e17 ? -1 : r;
}

static bool validPath(const Grid& g, const std::vector<Cell>& p, Cell s, Cell t) {
  if (p.empty() || p.front() != s || p.back() != t) return false;
  for (size_t i = 0; i < p.size(); ++i) {
    if (!g.free(p[i].first, p[i].second)) return false;
    if (i == 0) continue;
    int dr = p[i].first - p[i - 1].first, dc = p[i].second - p[i - 1].second;
    if (std::abs(dr) > 1 || std::abs(dc) > 1 || (!dr && !dc)) return false;
    if (dr && dc && (!g.free(p[i - 1].first + dr, p[i - 1].second) || !g.free(p[i - 1].first, p[i - 1].second + dc)))
      return false;
  }
  return true;
}

static Grid fromStrings(const std::vector<std::string>& rows) {
  Grid g; g.rows = rows.size(); g.cols = rows[0].size();
  for (auto& s : rows) for (char ch : s) g.occ.push_back(ch == '#');
  return g;
}

int main() {
  CHECK_NEAR(pathCost({{0, 0}, {1, 1}, {1, 2}}), std::sqrt(2.0) + 1.0, 1e-12);

  Grid open = fromStrings({".....", ".....", ".....", ".....", "....."});
  auto p = astar(open, {0, 0}, {4, 4});
  CHECK(validPath(open, p, {0, 0}, {4, 4}));
  CHECK_NEAR(pathCost(p), 4 * std::sqrt(2.0), 1e-9);

  Grid wall = fromStrings({
      ".......",
      ".#####.",
      ".....#.",
      "####.#.",
      ".......",
  });
  auto w = astar(wall, {2, 0}, {4, 0});
  CHECK(validPath(wall, w, {2, 0}, {4, 0}));
  CHECK_NEAR(pathCost(w), dijkstra(wall, {2, 0}, {4, 0}), 1e-9);

#ifdef HIDDEN
  // corner cutting forbidden: diagonal squeeze between two blocks
  Grid squeeze = fromStrings({
      ".#.",
      "#..",
      "...",
  });
  auto sq = astar(squeeze, {0, 0}, {2, 2});
  CHECK(sq.empty());   // (0,0) is boxed in: its only free neighbour (1,1) needs a corner cut
  Grid cc = fromStrings({"..", "#."});
  auto c2 = astar(cc, {1, 1}, {0, 0});
  CHECK(validPath(cc, c2, {1, 1}, {0, 0}));
  CHECK_NEAR(pathCost(c2), 2.0, 1e-12);  // must go around, not diagonally

  // trivial / degenerate
  CHECK(astar(open, {2, 2}, {2, 2}) == std::vector<Cell>({{2, 2}}));
  CHECK(astar(wall, {1, 1}, {0, 0}).empty());   // start occupied
  CHECK(astar(open, {0, 0}, {5, 5}).empty());   // goal out of bounds
  Grid cut = fromStrings({"..#..", "..#..", "..#.."});
  CHECK(astar(cut, {0, 0}, {2, 4}).empty());    // unreachable

  // random grids vs Dijkstra
  std::mt19937 rng(11);
  bool allOk = true;
  for (int trial = 0; trial < 15; ++trial) {
    Grid g; g.rows = 40; g.cols = 50; g.occ.resize(g.rows * g.cols);
    std::bernoulli_distribution occ(0.28);
    for (auto& x : g.occ) x = occ(rng);
    Cell s{0, 0}, t{39, 49};
    g.occ[0] = 0; g.occ.back() = 0;
    double ref = dijkstra(g, s, t);
    auto path = astar(g, s, t);
    if (ref < 0) allOk = allOk && path.empty();
    else allOk = allOk && validPath(g, path, s, t) && std::abs(pathCost(path) - ref) < 1e-9;
  }
  CHECK(allOk);
#endif
  return TEST_SUMMARY();
}
