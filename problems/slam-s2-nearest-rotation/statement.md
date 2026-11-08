# Re-orthonormalize a Drifting Rotation

After thousands of gyro integration steps $R \leftarrow R\,\exp(\omega\,\Delta t)$, floating-point errors have pushed `R` off SO(3): its columns are no longer exactly orthonormal. Project it back.

```cpp
// The rotation matrix closest to M in Frobenius norm: argmin_{R in SO(3)} ||R - M||_F
Eigen::Matrix3d nearestRotation(const Eigen::Matrix3d& M);
```

### Requirements
- The output must satisfy $R^TR = I$ and $\det R = +1$ (≤ 1e-12), even when $\det M < 0$.
- It must be the **Frobenius-nearest** rotation, not just *some* orthonormal matrix (Gram-Schmidt is not accepted on the hidden tests).
- If M is already a rotation, or a positive multiple of one, return that rotation.
- You may assume M has rank ≥ 2.

### Think about
- Why is Gram-Schmidt biased (which axis does it trust)?
- How would you renormalize if the state were a quaternion? Which is cheaper?
- The same "SVD + det fix" step appears in Kabsch/Umeyama point-set registration. Why?
- If the singular values are (3, 2, 1) and $\det(UV^T)=-1$, why flip the *last* singular vector?
