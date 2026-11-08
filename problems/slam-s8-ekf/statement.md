# EKF Localization with Range-Bearing Landmarks

A differential-drive robot localizes against **known** landmarks with a 2D lidar feature extractor. Implement the predict and update steps of an EKF over the state $\mathbf x = (x, y, \theta)$.

```cpp
struct EkfState {
  Eigen::Vector3d x;   // (x, y, theta), theta in (-pi, pi]
  Eigen::Matrix3d P;   // covariance
};
double wrapAngle(double a);                                    // -> (-pi, pi]
void ekfPredict(EkfState& s, double v, double w, double dt, const Eigen::Matrix3d& Q);
void ekfUpdate(EkfState& s, const Eigen::Vector2d& z,          // z = (range, bearing)
               const Eigen::Vector2d& landmark, const Eigen::Matrix2d& R);
```

### Given models
- **Motion** (Euler unicycle, using the *current* $\theta$):
  $x' = x + v\,dt\cos\theta,\; y' = y + v\,dt\sin\theta,\; \theta' = \theta + \omega\,dt$. Process noise $Q$ is added in state space: $P' = FPF^\top + Q$.
- **Measurement** to landmark $m$: $h(\mathbf x) = \big(\lVert m - p\rVert,\ \operatorname{atan2}(m_y - y, m_x - x) - \theta\big)$, noise $R$.

You derive the Jacobians.

### Requirements
- After both `ekfPredict` and `ekfUpdate`, $\theta$ must be wrapped to $(-\pi, \pi]$.
- The bearing component of the innovation must be wrapped.
- $P$ must remain **symmetric positive definite** after many updates.
- You may assume the landmark is not at the robot position.

### Think about
- How would you gate outlier measurements (Mahalanobis distance, $\chi^2$ threshold)?
- EKF-SLAM: how does the state grow when landmarks are unknown, and why is each update $O(n^2)$?
- Why is the EKF inconsistent (overconfident) over long runs? Why do VIO systems use an **error-state** EKF?
- UKF: when is it worth the extra sigma points?
