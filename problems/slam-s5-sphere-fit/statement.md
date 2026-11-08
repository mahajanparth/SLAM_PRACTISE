# Sphere Fit: algebraic least squares + RANSAC

Calibration targets (spheres) and pole/trunk detection (circles) need a fast closed-form fit. Then you need a robust version.

```cpp
using Cloud = std::vector<Eigen::Vector3d>;

bool fitSphere(const Cloud& pts, Eigen::Vector3d& center, double& radius);

bool ransacSphere(const Cloud& pts, double thresh, int iters,
                  Eigen::Vector3d& center, double& radius, std::vector<int>& inliers,
                  unsigned seed = 42);
```

**`fitSphere`**: the *algebraic* least-squares fit. Find a **linear** formulation of $\lVert p_i - c\rVert^2 = r^2$ and solve it in one shot (no iterations).
Return `false` if there are fewer than 4 points or the problem is degenerate (e.g. all points coplanar).

**`ransacSphere`**: minimal samples of 4 distinct points. A point is an inlier if $\big|\,\lVert p_i - c\rVert - r\,\big| \le$ `thresh`. Return the best model after a final `fitSphere` refit on its inliers, with `inliers` = **ascending** indices consistent with the returned model. Use your own `std::mt19937(seed)`. Return `false` if there are fewer than 4 points or no valid sample was found.

### Constraints
- Up to ~10⁴ points; the sphere may be far from the origin and only partially observed (a cap).
- `fitSphere` is $O(N)$; `ransacSphere` is $O(\text{iters}\cdot N)$.

### Think about
- What does the algebraic fit minimise, and how does it differ from the geometric error $\sum(\lVert p-c\rVert - r)^2$? How would you refine it (Gauss-Newton)?
- Why does centring the data improve conditioning?
- Same trick in 2D for a circle (trunk/pole detection from a lidar ring)? What about a cylinder?
