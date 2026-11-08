# Rotation Matrix ↔ Quaternion (by hand)

Your state estimator stores orientation as a quaternion, but a calibration file gives you a rotation matrix (and vice versa). Implement both conversions **without** Eigen's `Quaternion` constructors or `toRotationMatrix()`.

```cpp
Eigen::Vector4d rotToQuat(const Eigen::Matrix3d& R);   // returns (w, x, y, z), Hamilton convention
Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q);   // q = (w, x, y, z), not necessarily unit
```

### Requirements
- `rotToQuat`: `R` is a proper rotation (orthonormal, det = +1, up to ~1e-12 noise). Return a **unit** quaternion with **w ≥ 0**. When w = 0 (exactly 180°), either sign of the vector part is accepted.
- `quatToRot`: `q` may be non-unit (but non-zero). Normalize it first, so the result is always a rotation.
- Must be accurate (≤ 1e-9) for **every** rotation, including 180° rotations about any axis.
- Convention: $R(q)\,v$ equals the Hamilton product $q\,v\,q^{*}$, the same as Eigen's `Quaterniond(w,x,y,z).toRotationMatrix()` (which the tests use as ground truth).

### Think about
- Why does the "obvious" formula $w=\tfrac12\sqrt{1+\operatorname{tr}R}$, $x=(R_{21}-R_{12})/4w$ fail, and for which rotations?
- Double cover: $q \equiv -q$. Why canonicalize $w\ge0$, and where does that still leave an ambiguity?
- Hamilton vs JPL convention: what changes?
- Eigen's `Quaterniond(w,x,y,z)` constructor order vs `coeffs()` storage order: which one is (x,y,z,w)?
