# Least-Squares Plane & Line Fit

After RANSAC picks the inliers, refit the model by **total least squares**, minimising the *orthogonal* distances.

```cpp
using Cloud = std::vector<Eigen::Vector3d>;

bool fitPlane(const Cloud& pts, Eigen::Vector3d& normal, double& d);
bool fitLine (const Cloud& pts, Eigen::Vector3d& point, Eigen::Vector3d& dir);
```

| Function | Model | Return `false` when |
|---|---|---|
| `fitPlane` | $n\cdot p + d = 0$, $\lVert n\rVert=1$, minimising $\sum_i (n\cdot p_i + d)^2$ | fewer than 3 points, or the points are collinear or identical (no unique plane) |
| `fitLine` | $p(s) = \text{point} + s\,\text{dir}$, $\lVert\text{dir}\rVert=1$, minimising the sum of squared distances to the line | fewer than 2 points, or all points identical |

- Either sign of `normal` / `dir` is accepted. For the line, `point` may be any point on the line.
- Use a *relative* degeneracy test (scale-invariant): e.g. collinear if $\lambda_1 \le 10^{-10}\,\lambda_2$.
- $O(N)$.

### Think about
- Why not regress $z = ax + by + c$?
- Why must you centre the points? What happens numerically at map coordinates ~10⁴ m if you accumulate $\sum pp^\top$ and subtract $N\mu\mu^\top$?
- The smallest eigenvalue over $N$ is the mean squared residual. How do you use it as a planarity or "curvature" score?
- SVD of the $N\times3$ centred matrix vs eigen-decomposition of the $3\times3$ covariance: which is cheaper and which is more accurate?
