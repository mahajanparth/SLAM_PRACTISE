# Sensor Sync: timestamp matching & ground split

Your perception stack fuses a 10 Hz lidar with a 30 Hz camera. For every lidar sweep you need the camera frame closest in time, and before clustering you split the cloud into ground and non-ground points without losing the scan order (ring/azimuth order matters downstream).

```cpp
struct Point3 { double x, y, z; };
```

| Function | Behaviour |
|---|---|
| `std::vector<std::pair<std::size_t, std::size_t>> matchTimestamps(const std::vector<double>& lidar, const std::vector<double>& cam, double max_dt)` | Both vectors are sorted ascending (duplicates possible). For each lidar index `i`, find the camera index `j` minimising $\lvert lidar_i - cam_j\rvert$. On a tie, choose the **smaller** `j`. Emit `(i, j)` only if that minimum is ≤ `max_dt`. Output is in increasing `i`. Several lidar stamps may match the same camera frame |
| `std::size_t partitionGround(std::vector<Point3>& cloud, double z_thresh)` | Reorder `cloud` in place so all ground points (`z < z_thresh`) come first, then the rest. Both groups keep their **original relative order**. Return the number of ground points |

### Constraints
- n, m ≤ 2·10⁵. `matchTimestamps` in O(n log m) or O(n + m). `partitionGround` in O(n) (O(n log n) is acceptable without spare memory).

### Think about
- What exactly do `lower_bound` and `upper_bound` return? Which one would you use for "last stamp ≤ t"?
- What happens if you call `lower_bound` on an unsorted vector?
- Binary search per query vs a two-pointer sweep: when is each better?
- `partition` vs `stable_partition`: complexity and memory. Why might a real-time system avoid `stable_partition`?
- Floating-point timestamps vs `int64_t` nanoseconds: which would you store, and why?
