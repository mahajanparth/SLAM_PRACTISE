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
  // TODO
  return {};
}

double pathCost(const std::vector<Cell>& path) {
  // TODO
  return 0.0;
}
