# Batch Transform a Point Cloud

Every lidar scan has to be moved into the map frame: $p_w = T_{ws}\,p_s$ for ~100k points, 10–20 times a second. Do it efficiently for the three layouts you meet in practice.

```cpp
// 3xN, one point per column
Eigen::Matrix3Xd transformCloud(const Eigen::Isometry3d& T, const Eigen::Matrix3Xd& P);
// array of points
std::vector<Eigen::Vector3d> transformCloud(const Eigen::Isometry3d& T, const std::vector<Eigen::Vector3d>& pts);
// raw driver buffer: n records of 4 floats (x, y, z, intensity), packed. Return the transformed xyz as 3xN floats.
Eigen::Matrix3Xf transformRaw(const Eigen::Isometry3d& T, const float* xyzi, int n);
```

### Requirements
- Output column/element `i` is the transformed input point `i` (order preserved). Empty input gives empty output.
- No per-point heap allocation. The `Matrix3Xd` version should be loop-free (a single matrix product).
- `transformRaw` must **not** copy the input into an intermediate container. View it in place (intensity is skipped).
- Precision: double versions ≤ 1e-12, float version ≤ 1e-4 for coordinates up to 100 m.

### Think about
- Time the three variants for 1M points with `-O2`. Which wins, and why (GEMM, SIMD, memory bandwidth)?
- float vs double: memory, speed, and precision at 1 km from the origin.
- Row-major N×3 sensor layouts: how would you `Map` them?
- Why is `std::vector<Eigen::Vector4f>` sometimes used instead of `Vector3f`?
