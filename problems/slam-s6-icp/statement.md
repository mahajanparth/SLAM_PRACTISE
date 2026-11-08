# Point-to-Point ICP

Register a lidar scan `src` to a reference cloud `tgt` (the previous scan or the local map), starting from an initial guess such as a constant-velocity prediction.

```cpp
using Cloud = std::vector<Eigen::Vector3d>;

Eigen::Isometry3d icp(const Cloud& src, const Cloud& tgt, const Eigen::Isometry3d& init,
                      int max_iters,
                      double max_corr_dist = std::numeric_limits<double>::infinity());
```

Return $T$ such that $T\cdot\text{src}_i \approx$ its nearest neighbour in `tgt`, i.e. it locally minimises
$$E(T) = \sum_i \min_j \lVert T s_i - t_j\rVert^2 \quad\text{over pairs with distance} \le \texttt{max\_corr\_dist}.$$

- Start from `init`. Run at most `max_iters` iterations, each one: correspondences, then a closed-form rigid update. You may stop earlier once converged.
- Correspondences farther than `max_corr_dist` (measured with the current estimate) are ignored. If fewer than 3 pairs remain, stop and return the current estimate.
- If `src` or `tgt` is empty, or `max_iters <= 0`, return `init`.
- Write your own nearest-neighbour search (brute force $O(NM)$ is fine here) and your own Kabsch (no `Eigen::umeyama`).

### Constraints
A few hundred points per cloud. Tests use moderate motions (≈10°, 0.2 m) or a good `init`.

### Think about
- Why does ICP need a good initial guess? What happens at 90°?
- Convergence criteria: $\Delta$pose vs $\Delta$error. Which one do you trust?
- Outlier handling: distance gating, trimmed ICP, robust kernels (Huber/Cauchy), normal compatibility.
- Coarse-to-fine voxel pyramids. Real-time budget for a 100k-point scan at 10 Hz.
- How do you detect degeneracy (long corridor, open field)?
