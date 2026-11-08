# Two-View Triangulation

A visual SLAM front-end has matched a feature in two keyframes. Initialize the 3D map point.

Poses are **world→camera** transforms $T_{cw}$ (so $X_c = R X_w + t$). Observations are **normalized** image coordinates $(x, y) = (X_c/Z_c, Y_c/Z_c)$, so intrinsics are already removed.

```cpp
// Linear (DLT) triangulation via SVD.
Eigen::Vector3d triangulateDLT(const Eigen::Isometry3d& T1_cw, const Eigen::Vector2d& x1,
                               const Eigen::Isometry3d& T2_cw, const Eigen::Vector2d& x2);
// Midpoint of the shortest segment between the two back-projected rays.
Eigen::Vector3d triangulateMidpoint(const Eigen::Isometry3d& T1_cw, const Eigen::Vector2d& x1,
                                    const Eigen::Isometry3d& T2_cw, const Eigen::Vector2d& x2);
// true iff X_w has depth > 0 in BOTH cameras.
bool cheiralityOK(const Eigen::Isometry3d& T1_cw, const Eigen::Isometry3d& T2_cw,
                  const Eigen::Vector3d& X_w);
// Angle (radians) between the rays C1->X and C2->X (C = camera centre in world).
double parallaxAngle(const Eigen::Isometry3d& T1_cw, const Eigen::Isometry3d& T2_cw,
                     const Eigen::Vector3d& X_w);
```

### Assumptions
- The rays are not parallel for the triangulation functions (you need non-zero baseline and parallax).
- Noise-free observations must be triangulated to within $10^{-6}$.

### Think about
- Why is the DLT result not the maximum-likelihood estimate? What would you minimize instead?
- With pixel coordinates, why should you apply Hartley normalization first?
- ORB-SLAM rejects new points with parallax $< 1°$. Why? What happens to depth uncertainty as the baseline shrinks?
- The DLT happily returns a point *behind* both cameras. When does that happen in practice?
