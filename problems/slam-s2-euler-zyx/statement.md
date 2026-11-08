# Euler ZYX (yaw-pitch-roll) and Gimbal Lock

Your IMU driver reports yaw/pitch/roll, and your map stores rotation matrices. Convert between them, and handle the case every interviewer asks about: pitch = ±90°.

```cpp
// R = Rz(yaw) * Ry(pitch) * Rx(roll)   (intrinsic Z-Y'-X'', i.e. aerospace / ROS rpy)
Eigen::Matrix3d eulerZYXToRot(double yaw, double pitch, double roll);
// returns (yaw, pitch, roll)
Eigen::Vector3d rotToEulerZYX(const Eigen::Matrix3d& R);
```

### Requirements
- Ranges: yaw, roll ∈ [−π, π], pitch ∈ [−π/2, π/2].
- `eulerZYXToRot(rotToEulerZYX(R)) == R` for **every** rotation `R` (≤ 1e-9).
- Away from the singularity the original angles must come back (up to 2π wrapping).
- **Gimbal lock:** if $\sqrt{R_{00}^2+R_{10}^2} < 10^{-9}$, set **roll = 0** and put the whole remaining rotation into yaw.
- Don't use `R.eulerAngles()` (and see "Think about" for why it surprises people).

### Think about
- At pitch = ±90°, which combination of yaw and roll is still observable? What does that do to an EKF that uses Euler angles as its state?
- Why must you never average or linearly interpolate Euler angles?
- `R.eulerAngles(2, 1, 0)` returns the first angle in $[0,\pi]$. How can that bite you?
- Intrinsic ZYX vs extrinsic XYZ: same matrix?
