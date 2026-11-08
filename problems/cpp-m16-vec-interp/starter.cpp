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
    // TODO
    return {};
  }
  Vec operator-(const Vec& o) const {
    // TODO
    return {};
  }
  Vec operator*(T s) const {
    // TODO
    return {};
  }
  T dot(const Vec& o) const {
    // TODO
    return T{};
  }
  double norm() const {
    // TODO
    return 0.0;
  }
};

template <typename T, std::size_t N>
Vec<T, N> operator*(T s, const Vec<T, N>& a) {
  // TODO
  return {};
}

using Vec3d = Vec<double, 3>;

struct Angle { double rad; };

template <typename T>
T interpolate(const T& a, const T& b, double t) {
  // TODO: a + (b - a) * t
  return a;
}

template <>
Angle interpolate<Angle>(const Angle& a, const Angle& b, double t) {
  // TODO: shortest arc, result wrapped to (-pi, pi]
  return a;
}

template <typename T>
T sampleAt(const std::vector<double>& stamps, const std::vector<T>& values, double t) {
  // TODO: binary search + interpolate, clamp outside, T{} if empty
  return T{};
}
