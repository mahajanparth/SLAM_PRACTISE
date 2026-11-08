# IMU Sliding Window

A VIO front-end keeps the most recent `window` seconds of 200 Hz IMU samples so it can preintegrate between camera frames and estimate the gyro bias online. Samples come in at one end and leave at the other, which makes this a job for `std::deque`.

```cpp
struct ImuSample { double t; double gx, gy, gz; };   // timestamp [s], angular rate [rad/s]

class ImuWindow {
 public:
  explicit ImuWindow(double window_sec);
  bool push(const ImuSample& s);
  std::size_t size() const;
  std::pair<double, double> timeSpan() const;
  std::array<double, 3> meanGyro() const;
  std::vector<ImuSample> between(double t0, double t1) const;
};
```

| Member | Behaviour |
|---|---|
| `push(s)` | If `s.t` is **older** than the newest stored sample, reject it and return `false` (equal timestamps are accepted). Otherwise append it, then evict every sample with `t < s.t − window_sec`, and return `true` |
| `size()` | Number of stored samples |
| `timeSpan()` | `{oldest.t, newest.t}`, or `{0, 0}` when empty |
| `meanGyro()` | Mean of `gx, gy, gz` over the stored samples in **O(1)** (keep running sums). `{0,0,0}` when empty |
| `between(t0, t1)` | Copies of the stored samples with `t0 ≤ t ≤ t1`, in time order. Empty if none or if `t0 > t1` |

### Constraints
- Up to 2·10⁵ pushes. `push` is amortised O(1), `between` is O(log n + k).

### Think about
- Why not `std::vector` + `erase(begin())`? And why not `std::list`? (Cache behaviour, per-node allocation.)
- How is `std::deque` laid out in memory? Which operations invalidate its iterators, and which invalidate its references?
- Running sums drift over hours of floating-point add/subtract. How would you bound that?
- How does this compare with a fixed-capacity ring buffer over `std::array`?
