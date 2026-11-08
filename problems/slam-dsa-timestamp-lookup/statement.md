# Pose at a Lidar Timestamp

Odometry publishes poses at 100 Hz into a vector **sorted by timestamp**. A lidar scan arrives with stamp
`t`, and you need the robot pose at that instant.

```cpp
struct StampedPose {
  double t;                  // seconds
  Eigen::Vector3d p;         // position
  Eigen::Quaterniond q;      // orientation (unit)
};

std::optional<StampedPose> nearestPose(const std::vector<StampedPose>& traj, double t, double max_gap);
std::optional<StampedPose> interpolatePose(const std::vector<StampedPose>& traj, double t, double max_gap);
```

| Function | Behaviour |
|---|---|
| `nearestPose` | Sample with the closest stamp. On a tie, take the **earlier** one. `nullopt` if `traj` is empty or the closest sample is more than `max_gap` seconds away. |
| `interpolatePose` | Pose at exactly `t`: linear interpolation of `p`, **slerp** of `q`. The returned `t` equals the query. `nullopt` if `traj` is empty, `t` is outside `[traj.front().t, traj.back().t]` (no extrapolation), or the two bracketing samples are more than `max_gap` apart. A query exactly on a sample returns that sample. |

### Constraints
- Up to 10⁵ samples, many queries: each query must be **O(log n)**.
- Stamps are strictly increasing.

### Think about
- Why is lerping quaternion components followed by normalizing *almost* fine at 100 Hz, but wrong in general?
- `q` and `-q` are the same rotation. What does slerp do if the two samples have opposite signs?
- What if the lidar stamp marks the *start* of a 100 ms sweep? (per-point deskewing)
- Extrapolate past the last pose, or wait for the next one? (latency vs accuracy)
