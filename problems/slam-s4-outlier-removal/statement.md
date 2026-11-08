# Statistical Outlier Removal

Rain, dust and multipath reflections leave isolated "floating" points in lidar scans. Remove points whose neighbourhood is unusually sparse compared to the rest of the cloud.

```cpp
// Given (already in the starter): brute-force kNN, sorted by distance, ties -> smaller index.
std::vector<int> knn(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector3d& q, int k);

// Returns the indices of the INLIERS, in increasing order.
std::vector<int> statisticalOutlierRemoval(const std::vector<Eigen::Vector3d>& pts, int k, double alpha);
```

### Semantics
1. For every point $i$: $\bar d_i$ = mean Euclidean distance to its $k$ nearest **other** points (exclude the point itself by index; duplicates of it at distance 0 still count as neighbours). Use $k_{\text{eff}} = \min(k, N-1)$.
2. $\mu$ = mean of all $\bar d_i$, $\sigma$ = **population** standard deviation (divide by $N$).
3. Point $i$ is an inlier iff $\bar d_i \le \mu + \alpha\,\sigma$.
- `N <= 1` or `k <= 0`: everything is an inlier. Empty input gives `{}`.
- O(N² log k) with the given kNN is fine for the test sizes (≤ 2000 points).

### Think about
- What happens when all points are identical ($\sigma = 0$)? Why does `≤` matter?
- Radius outlier removal (at least m neighbours within r): when is it better? Which is cheaper with a KD-tree?
- A dense cluster of 20 dust points: will SOR remove it? Why not?
- Complexity with a KD-tree. Could you do it on a range image instead?
