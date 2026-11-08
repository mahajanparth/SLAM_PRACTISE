# Skew-Symmetric Matrix (hat) and vee

Cross products show up everywhere in SLAM (SO(3) exponential, rotation Jacobians, point-to-plane residuals), and they are almost always written as a matrix product $[\mathbf a]_\times \mathbf b = \mathbf a \times \mathbf b$. Implement the **hat** (skew) map and its inverse, **vee**.

```cpp
Eigen::Matrix3d skew(const Eigen::Vector3d& v);   // [v]x, so that skew(a) * b == a.cross(b)
Eigen::Vector3d vee(const Eigen::Matrix3d& S);    // inverse of skew: vee(skew(v)) == v
```

### Requirements
- `skew(v)` must be skew-symmetric: $[\mathbf v]_\times^T = -[\mathbf v]_\times$.
- `vee(S)` may assume `S` is (approximately) skew-symmetric. If it isn't exactly, return the vee of its skew-symmetric part $\tfrac12(S - S^T)$.
- No heap allocation, O(1).

### Think about
- Where does `skew` show up in SLAM? (SO(3) exp/log; the Jacobian of $R\,p$ w.r.t. a rotation perturbation is $-R[p]_\times$.)
- Why pass fixed-size Eigen types by `const&` rather than by value (alignment rules before C++17)?
- What are the eigenvalues of $[\mathbf v]_\times$? What is $[\mathbf v]_\times^2$ in terms of $\mathbf v\mathbf v^T$?
