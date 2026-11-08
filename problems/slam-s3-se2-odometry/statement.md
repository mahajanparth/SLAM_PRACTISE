# SE(2) Composition & Wheel-Odometry Integration

A differential-drive robot reports forward speed $v$ and yaw rate $\omega$ every few milliseconds. Dead-reckon its 2D pose.

```cpp
struct Pose2 { double x = 0, y = 0, theta = 0; };
struct OdomSample { double v, omega, dt; };   // v [m/s], omega [rad/s], dt [s] > 0

double wrapAngle(double a);                       // map to [-pi, pi]
Pose2 compose(const Pose2& a, const Pose2& b);    // a ⊕ b: apply b in a's frame (like T_w_a * T_a_b)
Pose2 integrateOdometry(const Pose2& start, const std::vector<OdomSample>& samples);
```

### Requirements
- Inside each sample, `v` and `omega` are **constant** for `dt`, so the robot moves on an exact circular arc (or a straight line if ω = 0). Integrate **exactly** (no Euler approximation). A single long sample must give the exact arc.
- Must be robust for ω = 0 and |ω| tiny (1e-12): no NaN/inf.
- `theta` of every returned pose is wrapped to $[-\pi,\pi]$.
- Empty `samples` returns `start` (wrapped).

### Think about
- How large is the error of forward-Euler integration on a circle? Midpoint (RK2) integration?
- Why must angle **differences** also be wrapped (e.g. in a residual)?
- Covariance propagation: what are the Jacobians $F=\partial x_{k+1}/\partial x_k$ and $G=\partial x_{k+1}/\partial(v,\omega)$? (This leads into the EKF.)
- How does `compose` relate to multiplying 3×3 homogeneous SE(2) matrices?
