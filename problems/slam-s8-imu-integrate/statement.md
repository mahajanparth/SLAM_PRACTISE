# IMU Strapdown Integration

Between two lidar frames you receive 200 Hz IMU samples. Propagate orientation, velocity and position in the world frame, given known biases.

```cpp
struct ImuState  { Eigen::Matrix3d R; Eigen::Vector3d v, p; };   // R = R_wb (body -> world)
struct ImuSample { double dt; Eigen::Vector3d gyro, accel; };    // body frame, rad/s and m/s^2

Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi);              // Rodrigues, safe near 0
ImuState integrateImu(ImuState s, const std::vector<ImuSample>& samples,
                      const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias,
                      const Eigen::Vector3d& gravity);           // e.g. (0, 0, -9.81)
```

### Given model (per sample, $\Delta t$ = `dt` of that sample)
The accelerometer measures **specific force** (a level IMU at rest reads $+9.81$ on z):
$$\hat\omega = \omega_m - b_g,\quad \hat a = a_m - b_a,\quad a_w = R\hat a + g$$
$$p \leftarrow p + v\Delta t + \tfrac12 a_w \Delta t^2,\qquad v \leftarrow v + a_w\Delta t,\qquad R \leftarrow R\,\mathrm{Exp}(\hat\omega\Delta t)$$
Compute all three right-hand sides from the state **before** the sample.

### Requirements
- `expSO3` must return an orthonormal matrix and handle $\lVert\phi\rVert \to 0$ without NaNs.
- `R` must stay a rotation ($R^\top R = I$, $\det R = 1$) after 10⁴ samples.

### Think about (discussion part of the interview)
- Why does VIO **preintegrate** ($\Delta R_{ij}, \Delta v_{ij}, \Delta p_{ij}$ in frame $i$) instead of integrating in the world frame?
- How are bias updates handled without re-integration (first-order bias Jacobians)?
- How does the covariance propagate, and why use an error-state formulation?
- Position error from a constant accel bias grows as $\tfrac12 b t^2$. What does 0.05 m/s² do over 10 s?
