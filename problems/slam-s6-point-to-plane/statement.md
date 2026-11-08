# Point-to-Plane ICP: one Gauss-Newton step

Point-to-plane ICP lets points slide along surfaces, so it converges in far fewer iterations than point-to-point on structured scenes. It's the core of most lidar odometry (LOAM's planar term, KISS-ICP variants, FAST-LIO's measurement model).

You're given correspondences that have already been transformed by the current estimate: source point $p_i$, target point $q_i$ and unit target normal $n_i$. Minimise
$$E(R, t) = \sum_i \big(n_i\cdot(R\,p_i + t - q_i)\big)^2$$
for a **small** motion, using the given linearisation $R \approx I + [\omega]_\times$, $t = v$, with unknown $\delta = (\omega, v)\in\mathbb R^6$.

```cpp
using Cloud = std::vector<Eigen::Vector3d>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// One Gauss-Newton step: returns δ = (ω, v) (rotation first).
Vector6d pointToPlaneStep(const Cloud& p, const Cloud& q, const Cloud& n);

// Applies the increment on the left: returns ΔT * T, where ΔT = (Exp(ω), v),
// i.e. rotation = AngleAxis(|ω|, ω/|ω|) and translation = v.
Eigen::Isometry3d applyUpdate(const Vector6d& delta, const Eigen::Isometry3d& T);
```

- Derive the per-correspondence residual and its $1\times6$ Jacobian, build the normal equations and solve them.
- Return the **zero vector** if the sizes differ, there are fewer than 6 correspondences, or the problem is degenerate: the smallest eigenvalue of the $6\times6$ normal matrix is $\le 10^{-9}\times$ the largest (e.g. all points on one plane).
- A pure-translation motion must be recovered **exactly** in one step (the problem is then linear).

### Constraints
$O(N)$ plus a $6\times6$ solve. Up to ~10⁴ correspondences.

### Think about
- Why does point-to-plane converge faster than point-to-point? What about GICP (plane-to-plane)?
- What does each eigenvector of $H$ mean physically? How do you detect a corridor, and what does LOAM's "solution remapping" do?
- Why apply the update with $\mathrm{Exp}(\omega)$ instead of $I+[\omega]_\times$?
- How would you add a robust kernel (Huber / Cauchy) with IRLS weights?
