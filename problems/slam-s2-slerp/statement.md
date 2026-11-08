# Quaternion Slerp

A spinning lidar takes 100 ms per sweep while the robot moves. To **deskew** the scan you need the sensor orientation at each point's timestamp, so you interpolate between the start and end orientations on the rotation manifold.

```cpp
// Spherical linear interpolation along the SHORTEST arc. t in [0, 1]; t=0 -> a, t=1 -> b.
Eigen::Quaterniond slerp(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double t);
```

### Requirements
- Inputs are unit quaternions. The output must be a unit quaternion.
- Don't use Eigen's `slerp` (the tests compare against it), `AngleAxis`, or `Quaternion::angularDistance`.
- Follow the shortest arc: `slerp(a, b, t)` and `slerp(a, -b, t)` represent the same rotation.
- Rotation angle is linear in `t`: the angle between `a` and the result is `t` times the angle between `a` and `b`.
- Must be robust when `a ≈ b` (no NaN) and when `a == -b`.
- Only the rotation matters (the result is compared up to sign).

### Think about
- Equivalent form: $a \exp(t \log(a^{-1} b))$. Why?
- Why does nlerp (normalized linear interpolation) not give constant angular velocity? When is it good enough?
- How would you interpolate a full SE(3) pose between two lidar scan timestamps?
- What happens to slerp for two rotations exactly 180° apart?
