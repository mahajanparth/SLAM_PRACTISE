# Transform a Plane (and a Line) Between Frames

Your mapper keeps the ground plane and wall planes as landmarks. A plane is stored as $\pi = (\mathbf n, d)$ with $\mathbf n^T\mathbf x + d = 0$. You know a plane in frame A and need it in frame B, given `T_B_A` ($x_B = T_{BA}\,x_A$).

```cpp
// plane_A = (nx, ny, nz, d) in frame A. Return the same plane expressed in frame B.
Eigen::Vector4d transformPlane(const Eigen::Isometry3d& T_B_A, const Eigen::Vector4d& plane_A);

// Line given by a point and a direction in frame A -> the same line in frame B.
struct Line3 { Eigen::Vector3d point, dir; };
Line3 transformLine(const Eigen::Isometry3d& T_B_A, const Line3& line_A);
```

### Requirements
- $\mathbf n$ need **not** be unit length. Keep the scale: the result for $(\lambda\mathbf n, \lambda d)$ must be $\lambda$ times the result for $(\mathbf n, d)$. This implies $\lVert\mathbf n_B\rVert = \lVert\mathbf n_A\rVert$.
- Every point on the plane/line in A, transformed to B, must lie on the returned plane/line.
- `dir` keeps its length; `point` is the transformed point.

### Think about
- Why do normals transform with $R$ but the offset changes with $\mathbf t$? What would change for a non-rigid (affine) transform? (Hint: $A^{-T}$.)
- Signed distance of a point to the plane: why is it invariant under the transform when $\lVert\mathbf n\rVert=1$?
- Plücker coordinates $(\mathbf u, \mathbf m = \mathbf p \times \mathbf u)$: how do they transform?
- Why are planes a popular landmark for indoor lidar/RGB-D SLAM (minimal parameterization issues)?
