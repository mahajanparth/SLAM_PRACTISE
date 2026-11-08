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
    // TODO: a good hash
    return 0;
  }
};

using OccMap = std::unordered_map<Cell, int, CellHash>;

Cell toCell(double x, double y, double res) {
  // TODO
  return {0, 0};
}

OccMap buildOccupancy(const std::vector<std::pair<double, double>>& hits, double res) {
  // TODO
  return {};
}

std::optional<double> planCost(const OccMap& occ, int min_hits, Cell start, Cell goal, Cell lo, Cell hi) {
  // TODO: Dijkstra on the 8-connected grid
  return std::nullopt;
}
