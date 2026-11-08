# `Vec<T, N>` and a Generic `interpolate<T>`

Your state estimator interpolates many kinds of quantities between timestamps: positions, velocities, and yaw angles.
Write the interpolation once as a template, then **specialize** it for angles, where linear interpolation is wrong
(the average of 179° and −179° is 180°, not 0°).

### 1. Fixed-size vector
```cpp
template <typename T, std::size_t N>
struct Vec {
  std::array<T, N> v{};                 // given
  T& operator[](std::size_t i);  const T& operator[](std::size_t i) const;   // given
  Vec operator+(const Vec& o) const;    // element-wise
  Vec operator-(const Vec& o) const;
  Vec operator*(T s) const;             // scale
  T dot(const Vec& o) const;
  double norm() const;                  // Euclidean length
};
template <typename T, std::size_t N> Vec<T, N> operator*(T s, const Vec<T, N>& a);   // s * a
using Vec3d = Vec<double, 3>;
```

### 2. Interpolation
```cpp
struct Angle { double rad; };   // yaw, any real value accepted as input

template <typename T> T interpolate(const T& a, const T& b, double t);            // a + (b - a) * t
template <> Angle interpolate<Angle>(const Angle& a, const Angle& b, double t);   // shortest arc
```
The `Angle` specialization moves from `a` towards `b` along the **shorter** arc and returns a value wrapped to
$(-\pi, \pi]$. If the two angles are exactly π apart, go counter-clockwise (the positive direction).

### 3. Sampling a time series
```cpp
template <typename T>
T sampleAt(const std::vector<double>& stamps, const std::vector<T>& values, double t);
```
`stamps` is strictly increasing and the same length as `values`. Return the interpolated value at time `t`. Clamp to
`values.front()` / `values.back()` outside the range, and return `T{}` if empty. Use `interpolate` (so a
`std::vector<Angle>` automatically uses the specialization). Target complexity: O(log n).

### Think about
- Why must the specialization be declared **before** `sampleAt` is instantiated?
- Why can't you partially specialize `interpolate` for every `Vec<T, N>`? What would you do instead?
- Quaternions: what does the specialization look like (slerp, and the `q` / `−q` sign check)?
- What happens to `Vec<int, 3>::operator*(T)` if you pass `0.5`?
