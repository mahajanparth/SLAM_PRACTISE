# Voxel Grid Downsample

Before registration, a 100k-point lidar scan is reduced by replacing all points in each occupied cube of side `leaf` with their **centroid**.

```cpp
std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& pts, double leaf);
```

### Semantics
- Voxel of point $p$: $(\lfloor p_x/\text{leaf}\rfloor, \lfloor p_y/\text{leaf}\rfloor, \lfloor p_z/\text{leaf}\rfloor)$. Voxels are half-open cells $[k\cdot\text{leaf}, (k+1)\cdot\text{leaf})$, and the grid is anchored at the origin.
- Output: one point per **occupied** voxel, the mean of the points in it. **Order is unspecified** (the tests sort).
- Coordinates may be negative and as large as ±1e4.
- Empty input gives empty output. `leaf <= 0` throws `std::invalid_argument`.
- Target O(N) expected time.

### Think about
- Centroid vs first point vs point closest to the voxel centre: trade-offs?
- What makes a good hash for 3 integers? What happens with a bad one (e.g. `x ^ y ^ z`)?
- Sort-based alternative: compute the keys, sort, reduce. When is it preferable (determinism, memory, cache)?
- How would you carry intensity/timestamp attributes? How would you build an incremental voxel map that keeps updating (iVox)?
- Memory budget for 1M voxels in `std::unordered_map`?
