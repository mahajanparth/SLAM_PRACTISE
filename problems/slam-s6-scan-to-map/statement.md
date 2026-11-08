# Scan-to-Map Odometry Skeleton

The design question that closes most lidar SLAM interviews: *"Sketch a lidar odometry pipeline."* The full pipeline looks like this:

```
scan ─► deskew (IMU / const-vel) ─► voxel downsample ─► register to LOCAL MAP (init = motion prior)
     ─► pose ─► keyframe? ─► insert into local map, drop far voxels
```

Registration itself is injected as a callback, so you can plug in the ICP you wrote earlier. Implement the **glue**: motion prior, keyframe test, local map, and orchestration.

```cpp
using Cloud = std::vector<Eigen::Vector3d>;

// Pose of scan k predicted from k-1 (T1) and k-2 (T2), assuming constant body-frame velocity.
Eigen::Isometry3d predictConstantVelocity(const Eigen::Isometry3d& T2, const Eigen::Isometry3d& T1);

// True iff the motion from the last keyframe has translation >= min_dist OR rotation angle >= min_angle.
bool isKeyframe(const Eigen::Isometry3d& T_kf, const Eigen::Isometry3d& T_cur, double min_dist, double min_angle);

class VoxelHashMap {                       // local map in WORLD coordinates
 public:
  VoxelHashMap(double voxel_size, int max_points_per_voxel);
  void addPoints(const Cloud& pts);        // voxel = floor(p / voxel_size); a full voxel ignores new points
  void removeFarFrom(const Eigen::Vector3d& origin, double max_dist);  // erase voxels whose FIRST point is > max_dist away
  Cloud pointCloud() const;                // all stored points, any order
  size_t numVoxels() const;
};

struct OdomParams { double voxel_size = 0.5; int max_points_per_voxel = 20;
                    double max_range = 50.0; double kf_dist = 1.0; double kf_angle = 0.2; };

class LidarOdometry {
 public:
  using RegisterFn = std::function<Eigen::Isometry3d(const Cloud& scan, const VoxelHashMap& map,
                                                     const Eigen::Isometry3d& init)>;
  LidarOdometry(const OdomParams& params, RegisterFn reg);
  Eigen::Isometry3d process(const Cloud& scan);   // returns the world pose of this scan
  const std::vector<Eigen::Isometry3d>& poses() const;
  const VoxelHashMap& map() const;
};
```

### `process(scan)` contract
1. **First scan:** pose = identity (don't call `reg`). Add the scan to the map; it's the first keyframe.
2. Otherwise `init` = `predictConstantVelocity(poses[k-2], poses[k-1])`, or `poses[k-1]` if only one pose exists. Then `T = reg(scan, map, init)`. Append `T` to `poses`.
3. If `isKeyframe(last_kf, T, kf_dist, kf_angle)`: add `T * scan` to the map, then `removeFarFrom(T.translation(), max_range)`, and set `last_kf = T`. Otherwise leave the map unchanged.

### Think about
- Why scan-to-map instead of scan-to-scan? What do KISS-ICP (voxel hash + adaptive threshold) and FAST-LIO2 (ikd-tree + iterated EKF with IMU) do differently?
- Where does deskewing go, and what does it need (per-point timestamps, a motion model)?
- Threading: a producer/consumer queue between the driver callback and odometry. What if processing takes longer than 100 ms?
- How would you detect registration failure (fitness, Hessian eigenvalues) and recover?
