# Bounded OccupancyGrid

A 2D occupancy grid stores the **log-odds** $l$ of each cell being occupied. The class must make it impossible to read or write outside the map, or to push a cell's log-odds outside the clamp range (which keeps cells from saturating, so the map can still respond to change). Types: `struct Vec2 { double x, y; };` and `struct Cell { int x, y; };`. The grid is `width × height` cells with side `resolution` (metres), and cell `(0,0)`'s lower-left corner is at world `(origin_x, origin_y)`.

| Member | Behaviour |
|---|---|
| `OccupancyGrid(int width, int height, double resolution, double origin_x, double origin_y)` | All cells start at `l = 0`. Throws `std::invalid_argument` unless `width > 0`, `height > 0` and `resolution > 0` (finite). |
| `static std::optional<OccupancyGrid> create(...)` (same params) | Non-throwing factory: `std::nullopt` on invalid parameters. |
| `int width() const`, `int height() const`, `double resolution() const` | Accessors. |
| `bool inBounds(int cx, int cy) const` | `0 ≤ cx < width` and `0 ≤ cy < height`. |
| `std::optional<Cell> worldToCell(double x, double y) const` | The cell containing the point, or `nullopt` if outside the grid. Cells are half-open: the cell `(i, j)` covers $[o_x + i\,r,\; o_x + (i+1)r)$, and the same for y. |
| `Vec2 cellCenter(int cx, int cy) const` | World coordinates of the cell centre. Throws `std::out_of_range` if out of bounds. |
| `double logOdds(int cx, int cy) const` | Throws `std::out_of_range` if out of bounds. |
| `void update(int cx, int cy, double delta)` | `l ← clamp(l + delta, kMinLogOdds, kMaxLogOdds)` with the constants `−4` and `4`. Throws `std::out_of_range` if out of bounds. |
| `double probability(int cx, int cy) const` | $p = 1 - \frac{1}{1 + e^{l}}$. Throws `std::out_of_range` if out of bounds. |
| `std::size_t countOccupied(double p_thresh) const` | Number of cells with $p >$ `p_thresh`. |

### Constraints
- Grids up to 1000 × 1000. All per-cell operations are O(1). Origins may be negative.
- The class must be copyable and movable with the defaults (rule of zero).

### Think about
- Why do `(int)(-0.5)` and `std::floor(-0.5)` differ, and why does it matter for a map centred on the robot?
- Exceptions vs `std::optional` factory: which one would you choose on an embedded target built with `-fno-exceptions`?
- Why clamp log-odds at all? What happens to a cell that has been observed "occupied" 10 000 times, and then a door opens?
- Row-major `cy * width + cx`: which loop order is cache-friendly when you raytrace or blur the map?
