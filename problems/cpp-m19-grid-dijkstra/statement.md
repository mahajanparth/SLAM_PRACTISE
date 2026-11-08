# Hashed Occupancy Grid + Dijkstra Planner

A local planner builds a sparse 2-D occupancy grid from lidar hits (only occupied cells are stored) and then runs
Dijkstra over an 8-connected grid.

```cpp
struct Cell { int x, y; bool operator==(const Cell& o) const; };     // given
struct CellHash { std::size_t operator()(const Cell& c) const noexcept; };
using OccMap = std::unordered_map<Cell, int, CellHash>;             // cell -> number of hits
```

| Function | Behaviour |
|---|---|
| `CellHash::operator()` | A good hash. Nearby cells must spread well: `(a,b)` and `(b,a)` must differ, and `(k,k)` must not all collide |
| `Cell toCell(double x, double y, double res)` | Grid index `floor(x/res), floor(y/res)` (note negative coordinates) |
| `OccMap buildOccupancy(const std::vector<std::pair<double,double>>& hits, double res)` | Count hits per cell |
| `std::optional<double> planCost(const OccMap& occ, int min_hits, Cell start, Cell goal, Cell lo, Cell hi)` | Length of the shortest path, or `nullopt` |

**Planner rules.** The search area is the inclusive box `lo.x ≤ x ≤ hi.x`, `lo.y ≤ y ≤ hi.y`. A cell is an **obstacle**
if it has `occ hits >= min_hits` (cells missing from the map have 0 hits). Moves go to the 8 neighbours, costing 1
(orthogonal) or √2 (diagonal). Diagonal moves are allowed whenever the destination cell is free (corner-cutting is fine).
Return `nullopt` if the start or goal is outside the box or is an obstacle, or if the goal is unreachable. Return `0` when `start == goal`.

### Constraints
- Search boxes up to ~200 × 200 cells. Target O(V log V).

### Think about
- Why can't you put `Cell` in an `unordered_map` without the hash *and* `operator==`?
- `std::priority_queue` has no decrease-key. What do you do instead?
- How would A* change this (heuristic, admissibility of the octile distance)?
- Dense `std::vector<int>` grid vs. hash map: when is each better?
