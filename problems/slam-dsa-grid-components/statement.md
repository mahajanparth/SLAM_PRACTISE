# Connected Free-Space Regions

A planner must know whether the goal is even reachable before running A\*. Given an occupancy grid,
label its connected **free** regions using **4-connectivity** (up, down, left, right; diagonals do NOT connect).

```cpp
// grid[r][c]: 0 = free, 1 = occupied. All rows have the same length.
std::vector<std::vector<int>> labelFreeRegions(const std::vector<std::vector<int>>& grid);
int countFreeRegions(const std::vector<std::vector<int>>& grid);
```

| Function | Returns |
|---|---|
| `labelFreeRegions` | Grid of the same shape. Occupied cells get `-1`. Free cells get a region id `0, 1, 2, …`, numbered in the order each region's **first cell appears in a row-major scan**. |
| `countFreeRegions` | Number of free regions (0 for an empty grid or a fully occupied one). |

### Constraints
- Up to 10⁶ cells; O(rows · cols) time. Must not overflow the stack on a large open map.

### Think about
- BFS vs recursive DFS vs union-find (two-pass labelling): trade-offs?
- 8-connectivity: what changes? Which one should a planner use if the robot cannot cut corners?
- The map updates one cell at a time. Can you maintain components incrementally? (union-find handles
  merges easily, but splits are hard)
- Inflating obstacles by the robot radius before labelling.
