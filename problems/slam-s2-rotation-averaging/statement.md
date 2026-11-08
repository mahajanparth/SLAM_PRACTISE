# Rotation Averaging

You calibrated an IMU-to-camera rotation 50 times, or you have N noisy orientation estimates of the same landmark-board. Fuse them into one orientation.

```cpp
// Weighted mean of unit quaternions. weights.size() == qs.size(), all weights > 0.
Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& qs,
                                      const std::vector<double>& weights);
```

### Requirements
- Inputs are unit quaternions with **arbitrary signs** (the same rotation may come as $q$ or $-q$).
- Return a unit quaternion. It's compared to the expected mean **as a rotation** (sign doesn't matter).
- Accepted means: the chordal L2 mean (Markley's eigenvector method) or the geodesic (Karcher) mean. For the test data (spread up to ~60°, symmetric noise) they agree within 1e-3 rad. Symmetric configurations must give the exact centre (≤ 1e-9).
- Empty input: return identity. One input: return it.
- Complexity: O(N).

### Think about
- Why does component-wise averaging fail even when all the rotations are nearly identical?
- When is "align signs to the first quaternion, then normalize the sum" good enough?
- Chordal vs geodesic mean: what does each minimize?
- One estimate is a 120° outlier. How would you make the average robust (median in the tangent space, Huber weights / IRLS)?
