# Frame Chains and Relative Pose

Your robot carries a lidar and a camera. The localization module gives you `T_world_robot`, calibration gives you `T_robot_lidar`, and a separate camera tracker gives you `T_world_camera`. Convention: `T_a_b` maps points from frame **b** to frame **a** ($p_a = T_{ab}\,p_b$).

```cpp
// 1. Express a lidar point in the camera frame.
Eigen::Vector3d lidarPointInCamera(const Eigen::Isometry3d& T_world_robot,
                                   const Eigen::Isometry3d& T_robot_lidar,
                                   const Eigen::Isometry3d& T_world_camera,
                                   const Eigen::Vector3d& p_lidar);
// 2. Relative motion between two robot poses: returns T_a_b.
Eigen::Isometry3d relativePose(const Eigen::Isometry3d& T_w_a, const Eigen::Isometry3d& T_w_b);
// 3. Noise-free odometry prediction: given T_w_a and the measured motion T_a_b, return T_w_b.
Eigen::Isometry3d predictPose(const Eigen::Isometry3d& T_w_a, const Eigen::Isometry3d& T_a_b);
```

### Requirements
- All inputs are rigid transforms. Return proper `Isometry3d`s (rotation part orthonormal).
- `predictPose(T_w_a, relativePose(T_w_a, T_w_b)) == T_w_b`.

### Think about
- How do you make the "subscripts cancel" rule visible in code (naming)?
- Which order is wrong but *looks* right when the robot is near the world origin?
- For an odometry factor in a pose graph, what is the residual between the measured $\hat T_{ab}$ and the current estimates $T_{wa}, T_{wb}$?
- If `lidarPointInCamera` is called for 100k points, what would you precompute?
