# Templates: `lerp` and `interpolateAt`

You have a time-stamped signal (wheel speeds, GPS positions, ...) and need its value at an arbitrary timestamp, e.g. the time a lidar scan was taken. Write it once, generically.

```cpp
template <typename T>
T lerp(const T& a, const T& b, double t);          // a + t (b - a); t = 0 -> a, t = 1 -> b

template <typename T>
T interpolateAt(const std::vector<std::pair<double, T>>& samples, double t);
```

It must work for `T = double`, `T = Eigen::Vector3d`, and any `T` with `+`, `-` and `* double`.

### `interpolateAt` semantics
- `samples` are sorted by strictly increasing time (`.first`).
- If `t` falls between two samples, linearly interpolate between them.
- If `t` is exactly a sample time, return that sample's value.
- **Clamp** outside the range: `t` before the first sample returns the first value, `t` after the last returns the last value.
- One sample: always return it. Empty `samples`: throw `std::invalid_argument`.
- Complexity: O(log n) per query.

### Think about
- Why is `lerp` wrong for rotations (quaternions or matrices)? What should you use instead?
- Clamp, extrapolate or throw outside the range: which would you choose for sensor fusion, and why?
- How would you restrict `T` with `if constexpr`, `static_assert`, or C++20 concepts?
- Why must `lerp` return `T` and not `auto` when `T` is an Eigen type?
