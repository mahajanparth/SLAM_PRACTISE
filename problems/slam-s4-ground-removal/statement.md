# Simple Ground Removal

Before clustering obstacles, remove the ground returns. Implement the naive approach and a slightly smarter one, then explain where each one fails.

```cpp
// Points are in the SENSOR frame, mounted sensor_height metres above flat ground (z up).
// Ground iff z < -sensor_height + threshold. Returns indices of NON-ground points, increasing.
std::vector<int> removeGroundHeight(const std::vector<Eigen::Vector3d>& pts, double sensor_height, double threshold);

// Grid min-z: split the XY plane into square cells of side `cell` (cell index = floor(x/cell), floor(y/cell)).
// Ground iff z - (minimum z among all points in the same cell) < threshold.
// Returns indices of NON-ground points, increasing.
std::vector<int> removeGroundGrid(const std::vector<Eigen::Vector3d>& pts, double cell, double threshold);
```

### Requirements
- Output indices in increasing order. Empty input gives `{}`.
- Negative x/y must use `floor` cells (the cell containing x = −0.1 differs from the one containing x = +0.1).
- `removeGroundGrid` is O(N) expected time.
- `cell <= 0` throws `std::invalid_argument`.

### Think about
- Why does the height threshold fail on slopes, ramps, and when the vehicle pitches under braking?
- When does grid min-z fail? (Think: a cell fully covered by a car roof, or overhanging tree branches.)
- Better options: RANSAC plane (→ S5), concentric-zone local planes (Patchwork++), slope between adjacent rings on a range image.
- How would you use IMU gravity to make the height threshold work while the vehicle pitches?
