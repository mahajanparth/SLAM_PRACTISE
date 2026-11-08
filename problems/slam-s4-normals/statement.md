# Surface Normals via PCA (+ orientation)

Point-to-plane ICP, ground segmentation and plane landmarks all need per-point surface normals. Estimate them from the k nearest neighbours and orient them toward the sensor.

```cpp
struct Normal {
  Eigen::Vector3d n = Eigen::Vector3d::Zero();   // unit normal (zero if invalid)
  double curvature = 0.0;                        // lambda0 / (lambda0 + lambda1 + lambda2)
  bool valid = false;
};
// Given (already in the starter): brute-force kNN, sorted by distance, ties -> smaller index.
std::vector<int> knn(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector3d& q, int k);

std::vector<Normal> estimateNormals(const std::vector<Eigen::Vector3d>& pts, int k,
                                    const Eigen::Vector3d& viewpoint);
```

### Semantics
- Output has one entry per input point, in the same order.
- Neighbourhood of `pts[i]` = `knn(pts, pts[i], k)` (it contains the point itself). The covariance uses the neighbourhood's own centroid.
- With eigenvalues $\lambda_0\le\lambda_1\le\lambda_2$ of the covariance, the normal is the eigenvector of $\lambda_0$, oriented so that $\mathbf n\cdot(\text{viewpoint}-\mathbf p_i)\ge 0$.
- **Invalid** (`valid=false`, `n` zero, curvature 0) if the neighbourhood has fewer than 3 points, or it is degenerate: $\lambda_2 = 0$ (identical points) or $\lambda_1 \le 10^{-10}\lambda_2$ (collinear).
- Must be accurate for clouds far from the origin (e.g. UTM-like coordinates around $10^6$–$10^7$ m).

### Think about
- Why the *smallest* eigenvalue? What do $\lambda_1$ and $\lambda_2$ tell you (linearity/planarity features in LOAM)?
- One-pass covariance ($E[pp^T] - \bar p\bar p^T$) vs two-pass: why does one-pass fail far from the origin?
- How does the choice of k (or a radius) trade noise against smoothing over edges?
- Complexity here is O(N² log k). What would you use for 100k points?
