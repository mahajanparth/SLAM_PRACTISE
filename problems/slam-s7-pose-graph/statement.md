# 2D Pose Graph Optimization

A robot drives a square loop. Integrating its wheel odometry drifts, and at the end place recognition finds a **loop closure** back to the start. Write the back-end: a Gauss-Newton pose-graph optimiser on SE(2).

```cpp
struct Pose2 { double x = 0, y = 0, theta = 0; };
struct Edge2 {
  int i = 0, j = 0;   // edge from pose i to pose j (valid indices)
  Pose2 z;            // measured pose of j expressed in frame i (i.e. z ≈ x_i^{-1} x_j)
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity();  // information matrix Ω (symmetric PD)
};
```

| Function | Behaviour |
|---|---|
| `Eigen::Vector3d edgeError(const Pose2& xi, const Pose2& xj, const Pose2& z)` | Residual $e_{ij}$ (below), with the angle wrapped to $[-\pi, \pi]$ |
| `double graphCost(const std::vector<Pose2>& poses, const std::vector<Edge2>& edges)` | $\sum_{ij} e_{ij}^\top \Omega_{ij}\, e_{ij}$ |
| `int optimizePoseGraph(std::vector<Pose2>& poses, const std::vector<Edge2>& edges, int max_iters = 20)` | In-place Gauss-Newton. Returns the number of iterations performed |

The residual, which is a given here (the Jacobians are your job):
$$e_{ij} = \begin{bmatrix} R_z^\top\big(R_i^\top (t_j - t_i) - t_z\big) \\ \mathrm{wrap}(\theta_j - \theta_i - \theta_z) \end{bmatrix}$$
where $R_i = R(\theta_i)$, $R_z = R(\theta_z)$, and $t = (x, y)$.

### Requirements
- **Gauge:** pose 0 is the anchor and must not move (tolerance $10^{-6}$).
- Per iteration, build a dense $H$ ($3N\times3N$) and $b$, solve $H\Delta = -b$ with a Cholesky-type solver, and apply $x \leftarrow x + \Delta$. Return every `theta` wrapped to $[-\pi, \pi]$.
- Stop when $\|\Delta\| < 10^{-9}$ or after `max_iters`. With fewer than 2 poses or no edges, do nothing and return 0.
- The initial guess may contain **unwrapped** headings, e.g. $\theta = 4.7$.
- $N \le 100$.

### Think about
- What is the sparsity structure of $H$ for odometry plus a few loop closures? Why do real solvers use sparse Cholesky with AMD/COLAMD ordering?
- What happens if you forget the gauge? What other ways are there to fix it?
- A false loop closure gets in: how do robust kernels or switchable constraints help?
- How would iSAM2 avoid re-solving the whole problem when a new pose arrives?
- This uses $x \leftarrow x+\Delta$ on $(x, y, \theta)$. How does a proper $\boxplus$ on SE(2) differ?
