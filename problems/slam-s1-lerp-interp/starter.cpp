#include <Eigen/Dense>
#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

template <typename T>
T lerp(const T& a, const T& b, double t) {
  // TODO
  return a;
}

template <typename T>
T interpolateAt(const std::vector<std::pair<double, T>>& samples, double t) {
  // TODO
  if (samples.empty()) return T{};
  return samples.front().second;
}
