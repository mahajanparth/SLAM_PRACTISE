#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

template <typename T, std::size_t N>
struct Vec {
  std::array<T, N> v{};

  T& operator[](std::size_t i) { return v[i]; }
  const T& operator[](std::size_t i) const { return v[i]; }

  Vec operator+(const Vec& o) const {
    Vec r;
    for (std::size_t i = 0; i < N; ++i) r.v[i] = v[i] + o.v[i];  // N is constexpr: trivially unrolled
    return r;
  }
  Vec operator-(const Vec& o) const {
    Vec r;
    for (std::size_t i = 0; i < N; ++i) r.v[i] = v[i] - o.v[i];
    return r;
  }
  Vec operator*(T s) const {
    Vec r;
    for (std::size_t i = 0; i < N; ++i) r.v[i] = v[i] * s;
    return r;
  }
  T dot(const Vec& o) const {
    T s{};
    for (std::size_t i = 0; i < N; ++i) s += v[i] * o.v[i];
    return s;
  }
  double norm() const { return std::sqrt(static_cast<double>(dot(*this))); }
};

// Free function so that `2.0 * v` works (the left operand isn't a Vec).
template <typename T, std::size_t N>
Vec<T, N> operator*(T s, const Vec<T, N>& a) { return a * s; }

using Vec3d = Vec<double, 3>;

struct Angle { double rad; };

inline double wrapPi(double x) {  // -> (-pi, pi]
  constexpr double kPi = 3.14159265358979323846;
  x = std::fmod(x + kPi, 2.0 * kPi);
  if (x <= 0.0) x += 2.0 * kPi;
  return x - kPi;
}

// Primary template: works for anything with +, -, and * double (double, Vec3d, Eigen vectors...).
template <typename T>
T interpolate(const T& a, const T& b, double t) { return a + (b - a) * t; }

// Full specialization: angles live on a circle, so interpolate along the shortest arc.
template <>
Angle interpolate<Angle>(const Angle& a, const Angle& b, double t) {
  const double d = wrapPi(b.rad - a.rad);  // in (-pi, pi]: exactly-opposite goes +pi (CCW)
  return Angle{wrapPi(a.rad + t * d)};
}

template <typename T>
T sampleAt(const std::vector<double>& stamps, const std::vector<T>& values, double t) {
  if (values.empty()) return T{};
  if (t <= stamps.front()) return values.front();
  if (t >= stamps.back()) return values.back();
  // first stamp > t; guaranteed to be in [1, n-1] by the clamps above
  const std::size_t k = static_cast<std::size_t>(std::upper_bound(stamps.begin(), stamps.end(), t) - stamps.begin());
  const double alpha = (t - stamps[k - 1]) / (stamps[k] - stamps[k - 1]);
  return interpolate(values[k - 1], values[k], alpha);  // T = Angle -> specialization
}
