# Kabsch / Umeyama: closed-form alignment

Given known correspondences (feature matches, ICP's inner step, GPS-to-odometry trajectory alignment), find the rigid transform that best aligns them.

```cpp
// Rigid: argmin_{R,t} Σ ||R p_i + t - q_i||²   (P, Q are 3×N, column i corresponds)
Eigen::Isometry3d kabsch(const Eigen::Matrix3Xd& P, const Eigen::Matrix3Xd& Q);

// Similarity: argmin_{s,R,t} Σ ||s R p_i + t - q_i||²; returns [sR t; 0 0 0 1]
Eigen::Matrix4d umeyamaSim3(const Eigen::Matrix3Xd& P, const Eigen::Matrix3Xd& Q);
```

- $R$ must be a **proper rotation** ($R^\top R = I$, $\det R = +1$), even when the unconstrained optimum would be a reflection.
- If `P.cols() != Q.cols()` or there are fewer than 3 correspondences, return identity.
- For degenerate (collinear) inputs $R$ is not unique. Return any proper rotation that achieves the minimum.
- Do **not** call `Eigen::umeyama`; that's what the tests compare against.

### Constraints
$O(N)$ plus one 3×3 SVD.

### Think about
- Why does a closed form exist at all (orthogonal Procrustes)?
- When does $\det(VU^\top) = -1$ happen with real data?
- Weighted version (per-point weights from a robust kernel): what changes?
- How many non-degenerate correspondences are needed, and why are 3 collinear points not enough?
- Where does Umeyama's scale show up in practice (monocular SLAM, aligning to GPS for ATE evaluation)?
