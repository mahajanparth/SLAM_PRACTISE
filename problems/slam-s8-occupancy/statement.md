# Occupancy Grid Mapping

Build a 2D occupancy map from a planar lidar. Each cell stores **log-odds** $\ell$ (0 = unknown, $p = 0.5$).

```cpp
std::vector<std::pair<int,int>> bresenham(int x0, int y0, int x1, int y1);

class OccupancyGrid {
 public:
  static constexpr double kLogFree = -0.4, kLogOcc = 0.85, kLogMin = -2.0, kLogMax = 3.5;
  OccupancyGrid(int width, int height, double resolution, const Eigen::Vector2d& origin);  // given
  bool inBounds(int cx, int cy) const;                                   // given
  double logOdds(int cx, int cy) const;                                  // given (0 if out of bounds)
  bool worldToCell(const Eigen::Vector2d& p, int& cx, int& cy) const;    // TODO
  double probability(int cx, int cy) const;                              // TODO
  void integrateScan(const Eigen::Vector3d& pose, const std::vector<double>& ranges,
                     double angle_min, double angle_increment, double max_range);  // TODO
};
```

### Semantics
- **`bresenham`**: all cells of the discrete line from $(x_0,y_0)$ to $(x_1,y_1)$ **inclusive**, ordered from start to end, consecutive cells 8-adjacent. Must work for every octant: steep, negative, and reversed slopes. Implement it yourself.
- **`worldToCell`**: cell $c = \lfloor (p - \text{origin}) / \text{res} \rfloor$. Writes `cx, cy` **always**, and returns whether the cell is inside the grid. Cell $(0,0)$ covers $[\text{origin}, \text{origin} + \text{res})$. Storage is row-major `data[cy * width + cx]`.
- **`probability`**: $p = 1 - 1/(1 + e^{\ell})$.
- **`integrateScan`**: the pose is $(x, y, \theta)$ in the world, and beam $i$ has angle $\theta + \text{angle\_min} + i\cdot\text{angle\_increment}$.
  - Ranges that are non-finite or $\le 0$ are ignored.
  - If $r < \text{max\_range}$, the beam is a **hit**. Trace the cells from the robot cell to the endpoint cell; every cell except the last gets `kLogFree` added, and the endpoint gets `kLogOcc`.
  - If $r \ge \text{max\_range}$ there is **no hit**. Trace to the point at `max_range`, and *all* traced cells (including the last) get `kLogFree`.
  - Cells outside the grid are skipped, but the ray still continues, and the beam may leave and stay outside the map.
  - Every beam is applied independently, and after each addition $\ell$ is clamped to $[\text{kLogMin}, \text{kLogMax}]$.

### Think about
- Why log-odds instead of probabilities? Why clamp?
- How do you handle dynamic objects (people walking through)?
- A 1 km × 1 km map at 5 cm resolution is 400 M cells. How would you store it (tiles, hashing, OctoMap for 3D)?
- Should a cell hit by many beams in one scan be updated many times?
