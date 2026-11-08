# Pose at the Lidar Timestamp

Wheel odometry arrives at 50 Hz, and lidar scans at 10 Hz with their own timestamps. To deskew or register a scan you need the
robot's pose **at the scan time**, and the motion between two scans.

```cpp
struct Pose2 { double x, y, theta; };          // world frame, theta in radians
struct StampedPose { double t; Pose2 pose; };  // trajectory sorted by strictly increasing t
```

| Function | Behaviour |
|---|---|
| `double wrapAngle(double a)` | Wrap to $(-\pi, \pi]$ |
| `std::optional<std::size_t> nearestIndex(const std::vector<StampedPose>& traj, double t, double max_dt)` | Index of the sample closest in time to `t`, ties → the **earlier** one; `nullopt` if empty or the closest is farther than `max_dt`. **O(log n)** |
| `std::optional<Pose2> poseAt(const std::vector<StampedPose>& traj, double t)` | Interpolated pose. `nullopt` if empty or `t` is outside `[traj.front().t, traj.back().t]` (no extrapolation). Position: linear. Heading: along the **shortest arc**, result wrapped. **O(log n)** |
| `Pose2 relative(const Pose2& a, const Pose2& b)` | $T_a^{-1} T_b$: pose of `b` expressed in `a`'s frame, theta wrapped |
| `std::optional<Pose2> motionBetween(const std::vector<StampedPose>& traj, double t0, double t1)` | `relative(poseAt(t0), poseAt(t1))`, or `nullopt` if either is unavailable |

The given SE(2) facts: $T_a^{-1} T_b$ has translation $R(-\theta_a)\,(p_b - p_a)$ and rotation $\theta_b - \theta_a$.

### Think about
- Why is "no extrapolation" safer than clamping? When would a short extrapolation be acceptable?
- Interpolating x, y, θ independently vs. interpolating on SE(2) (constant-twist arc): when does the difference matter?
- What would you do in 3-D (slerp; the `q` / `−q` sign check)?
- Odometry and lidar on different clocks: where does the time offset enter?
