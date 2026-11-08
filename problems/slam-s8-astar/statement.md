# A* on an 8-Connected Occupancy Grid

Plan a path for a ground robot on a 2D occupancy grid (row-major; `occ[r*cols + c] != 0` means occupied).

```cpp
struct Grid {
  int rows = 0, cols = 0;
  std::vector<uint8_t> occ;           // size rows*cols
  bool free(int r, int c) const;      // given: in bounds and not occupied
};
using Cell = std::pair<int, int>;     // (row, col)

std::vector<Cell> astar(const Grid& g, Cell start, Cell goal);
double pathCost(const std::vector<Cell>& path);   // sum of step costs (1 or sqrt(2)); 0 if size < 2
```

### Rules
- Moves go to the 8 neighbours. Straight moves cost $1$ and diagonal moves cost $\sqrt2$.
- **No corner cutting**: a diagonal move is allowed only if **both** orthogonally adjacent cells are free.
- Return the path **including** `start` and `goal`, in order. Return `{start}` if `start == goal`.
- Return `{}` if `start` or `goal` is blocked or out of bounds, or if the goal is unreachable.
- The path must be **optimal** (its cost equals Dijkstra's). Ties may be broken arbitrarily.
- Complexity: $O(N \log N)$ for $N$ cells.

### Think about
- When does A* reduce to Dijkstra? What happens with an inadmissible heuristic (e.g. Manhattan on an 8-grid)?
- How would you inflate obstacles by the robot radius?
- Paths on grids are jagged. What are the options: any-angle (Theta*), smoothing, or Hybrid-A* for car kinematics?
- Re-planning as the map updates: D* Lite.
