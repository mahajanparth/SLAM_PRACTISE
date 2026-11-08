# SE3 Struct: compose, inverse, act

Before reaching for `Eigen::Isometry3d`, show that you know what it does. Implement a minimal rigid-body transform $T=(R,\mathbf t)$ acting as $p \mapsto R\,p + \mathbf t$.

```cpp
struct SE3 {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  SE3 operator*(const SE3& other) const;                  // composition: (this * other)(p) = this(other(p))
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const;  // transform a point
  SE3 inverse() const;
  Eigen::Matrix4d matrix() const;                         // homogeneous 4x4 [R t; 0 0 0 1]
};
```

### Requirements
- `R` is always a proper rotation.
- `inverse()` must be O(1) and exact up to round-off, **without** inverting a 4×4 or 3×3 matrix.
- Semantics must match `Eigen::Isometry3d` (the tests compare against it).

### Think about
- `Eigen::Isometry3d` vs `Eigen::Affine3d`: why does `inverse()` cost more for one of them?
- Store `R` as a matrix or a quaternion? Trade-offs in memory, composition cost and drift.
- How would you add `log()`/`exp()` to make this a Lie group class (Sophus-style)?
- Why is `T * T.inverse()` only *approximately* the identity after millions of compositions?
