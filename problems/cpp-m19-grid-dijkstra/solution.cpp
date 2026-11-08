#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

struct Cell {
  int x, y;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

struct CellHash {
  std::size_t operator()(const Cell& c) const noexcept {
    // Pack both 32-bit coords losslessly into 64 bits, then mix (splitmix64 finalizer) so that
    // nearby cells land in different buckets even with power-of-two or prime bucket counts.
    std::uint64_t k = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(c.x)) << 32) |
                      static_cast<std::uint32_t>(c.y);
    k ^= k >> 30; k *= 0xbf58476d1ce4e5b9ULL;
    k ^= k >> 27; k *= 0x94d049bb133111ebULL;
    k ^= k >> 31;
    return static_cast<std::size_t>(k);
  }
};

using OccMap = std::unordered_map<Cell, int, CellHash>;

Cell toCell(double x, double y, double res) {
  // floor, not truncation: -0.1 belongs to cell -1, not 0
  return {static_cast<int>(std::floor(x / res)), static_cast<int>(std::floor(y / res))};
}

OccMap buildOccupancy(const std::vector<std::pair<double, double>>& hits, double res) {
  OccMap occ;
  occ.reserve(hits.size());
  for (const auto& [x, y] : hits) ++occ[toCell(x, y, res)];  // operator[] inserts 0 then increments
  return occ;
}

std::optional<double> planCost(const OccMap& occ, int min_hits, Cell start, Cell goal, Cell lo, Cell hi) {
  auto inBox = [&](const Cell& c) { return c.x >= lo.x && c.x <= hi.x && c.y >= lo.y && c.y <= hi.y; };
  auto blocked = [&](const Cell& c) {
    auto it = occ.find(c);  // find, never operator[] on the (const) map
    return it != occ.end() && it->second >= min_hits;
  };
  if (!inBox(start) || !inBox(goal) || blocked(start) || blocked(goal)) return std::nullopt;

  using Item = std::pair<double, Cell>;
  auto cmp = [](const Item& a, const Item& b) { return a.first > b.first; };  // min-heap on distance
  std::priority_queue<Item, std::vector<Item>, decltype(cmp)> pq(cmp);
  std::unordered_map<Cell, double, CellHash> dist;
  dist[start] = 0.0;
  pq.push({0.0, start});
  const double kDiag = std::sqrt(2.0);

  while (!pq.empty()) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) continue;           // stale entry (no decrease-key in priority_queue)
    if (u == goal) return d;             // first pop of the goal is optimal (non-negative weights)
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const Cell v{u.x + dx, u.y + dy};
        if (!inBox(v) || blocked(v)) continue;
        const double nd = d + ((dx != 0 && dy != 0) ? kDiag : 1.0);
        auto it = dist.find(v);
        if (it == dist.end() || nd < it->second) {
          dist[v] = nd;
          pq.push({nd, v});
        }
      }
    }
  }
  return std::nullopt;
}
