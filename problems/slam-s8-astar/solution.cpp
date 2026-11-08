#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

struct Grid {
  int rows = 0, cols = 0;
  std::vector<uint8_t> occ;  // row-major, nonzero = occupied
  bool free(int r, int c) const {
    return r >= 0 && c >= 0 && r < rows && c < cols && occ[static_cast<size_t>(r) * cols + c] == 0;
  }
};
using Cell = std::pair<int, int>;  // (row, col)

std::vector<Cell> astar(const Grid& g, Cell start, Cell goal) {
  if (!g.free(start.first, start.second) || !g.free(goal.first, goal.second)) return {};
  if (start == goal) return {start};

  const double kSqrt2 = std::sqrt(2.0);
  const double kInf = std::numeric_limits<double>::infinity();
  auto id = [&](int r, int c) { return r * g.cols + c; };
  auto h = [&](int r, int c) {  // octile distance: exact on an empty 8-grid
    const int dr = std::abs(r - goal.first), dc = std::abs(c - goal.second);
    return (std::max(dr, dc) - std::min(dr, dc)) + kSqrt2 * std::min(dr, dc);
  };

  const int n = g.rows * g.cols;
  std::vector<double> gcost(n, kInf);
  std::vector<int> parent(n, -1);
  std::vector<char> closed(n, 0);
  using QE = std::pair<double, int>;  // (f, id)
  std::priority_queue<QE, std::vector<QE>, std::greater<>> open;

  const int s = id(start.first, start.second), t = id(goal.first, goal.second);
  gcost[s] = 0.0;
  open.push({h(start.first, start.second), s});

  while (!open.empty()) {
    const int u = open.top().second;
    open.pop();
    if (closed[u]) continue;  // stale duplicate entry (lazy deletion)
    closed[u] = 1;
    if (u == t) break;
    const int r = u / g.cols, c = u % g.cols;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (!dr && !dc) continue;
        const int nr = r + dr, nc = c + dc;
        if (!g.free(nr, nc)) continue;
        const bool diag = dr && dc;
        if (diag && (!g.free(r + dr, c) || !g.free(r, c + dc))) continue;  // no corner cutting
        const int v = id(nr, nc);
        const double ng = gcost[u] + (diag ? kSqrt2 : 1.0);
        if (ng < gcost[v]) {
          gcost[v] = ng;
          parent[v] = u;
          open.push({ng + h(nr, nc), v});
        }
      }
    }
  }
  if (gcost[t] == kInf) return {};

  std::vector<Cell> path;
  for (int v = t; v != -1; v = parent[v]) path.emplace_back(v / g.cols, v % g.cols);
  std::reverse(path.begin(), path.end());
  return path;
}

double pathCost(const std::vector<Cell>& path) {
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const bool diag = path[i].first != path[i - 1].first && path[i].second != path[i - 1].second;
    cost += diag ? std::sqrt(2.0) : 1.0;
  }
  return cost;
}
