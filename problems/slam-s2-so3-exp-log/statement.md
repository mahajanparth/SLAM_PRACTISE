# SO(3) exp and log

Every modern SLAM optimizer updates rotations on the tangent space: $R \leftarrow R\,\exp(\delta\boldsymbol\theta)$. Implement the exponential and logarithm maps of SO(3).

```cpp
Eigen::Matrix3d skew(const Eigen::Vector3d& v);      // given (already implemented in the starter)
Eigen::Matrix3d expSO3(const Eigen::Vector3d& w);    // rotation vector (axis * angle) -> R
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);    // R -> rotation vector with |w| <= pi
```

### Requirements
- No Eigen `AngleAxis`, `Quaternion` or matrix `exp`/`log` (the tests use them as ground truth).
- `expSO3` must be accurate for **every** $\boldsymbol\omega$, including $\boldsymbol\omega=0$ and $\lVert\boldsymbol\omega\rVert = 10^{-9}$ (no NaN, error ≲ 1e-12).
- `logSO3` returns $\boldsymbol\omega$ with $\lVert\boldsymbol\omega\rVert\in[0,\pi]$, and must be accurate:
  - for tiny rotations (absolute error ≲ 1e-12),
  - for rotations at or very near **π** (when $\theta=\pi$ exactly, either $\pm\pi\mathbf a$ is accepted),
  - for inputs that are rotations up to rounding (e.g. trace slightly above 3).
- `logSO3(expSO3(w)) == w` for $\lVert w\rVert<\pi$.

### Think about
- Why optimize in the tangent space (minimal 3 parameters, no unit-norm/orthogonality constraint)?
- Right vs left perturbation: $R\exp(\delta)$ vs $\exp(\delta)R$. Which frame is $\delta$ in?
- What is the right Jacobian $J_r(\boldsymbol\omega)$, and where does it appear (IMU preintegration)?
- BCH: $\log(\exp(a)\exp(b)) \approx a + b + \tfrac12[a,b]$ for small $a,b$.
