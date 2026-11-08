#include <Eigen/Dense>
#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

// Returning T (not auto) forces Eigen expression templates to be evaluated.
template <typename T>
T lerp(const T& a, const T& b, double t) {
  return a + (b - a) * t;
}

template <typename T>
T interpolateAt(const std::vector<std::pair<double, T>>& samples, double t) {
  if (samples.empty()) throw std::invalid_argument("interpolateAt: no samples");
  if (t <= samples.front().first) return samples.front().second;  // clamp (also handles size 1)
  if (t >= samples.back().first) return samples.back().second;

  // First sample with time >= t. It can't be begin() (t > front) or end() (t < back).
  auto hi = std::lower_bound(samples.begin(), samples.end(), t,
                             [](const std::pair<double, T>& s, double v) { return s.first < v; });
  if (hi->first == t) return hi->second;  // exactly on a sample
  auto lo = std::prev(hi);
  const double alpha = (t - lo->first) / (hi->first - lo->first);
  return lerp(lo->second, hi->second, alpha);
}
