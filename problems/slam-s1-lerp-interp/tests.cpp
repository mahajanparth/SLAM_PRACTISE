// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  CHECK_NEAR(lerp(2.0, 4.0, 0.25), 2.5, 1e-12);
  CHECK_NEAR(lerp(2.0, 4.0, 1.0), 4.0, 1e-12);
  const Eigen::Vector3d a(0, 0, 0), b(2, -4, 6);
  CHECK_MAT_NEAR(lerp(a, b, 0.5), Eigen::Vector3d(1, -2, 3), 1e-12);

  std::vector<std::pair<double, double>> s = {{0.0, 10.0}, {1.0, 20.0}, {3.0, 0.0}};
  CHECK_NEAR(interpolateAt(s, 0.5), 15.0, 1e-12);
  CHECK_NEAR(interpolateAt(s, 2.0), 10.0, 1e-12);

  std::vector<std::pair<double, Eigen::Vector3d>> traj = {
      {10.0, Eigen::Vector3d(0, 0, 0)}, {10.1, Eigen::Vector3d(1, 0, 0)}, {10.3, Eigen::Vector3d(1, 2, 0)}};
  CHECK_MAT_NEAR(interpolateAt(traj, 10.2), Eigen::Vector3d(1, 1, 0), 1e-9);

#ifdef HIDDEN
  // clamp outside the range
  CHECK_NEAR(interpolateAt(s, -5.0), 10.0, 1e-12);
  CHECK_NEAR(interpolateAt(s, 99.0), 0.0, 1e-12);
  // exactly on samples (including the last one)
  CHECK_NEAR(interpolateAt(s, 1.0), 20.0, 1e-12);
  CHECK_NEAR(interpolateAt(s, 3.0), 0.0, 1e-12);
  CHECK_NEAR(interpolateAt(s, 0.0), 10.0, 1e-12);
  // single sample
  std::vector<std::pair<double, double>> one = {{5.0, 7.0}};
  CHECK_NEAR(interpolateAt(one, 4.0), 7.0, 0.0);
  CHECK_NEAR(interpolateAt(one, 6.0), 7.0, 0.0);
  // empty -> throws
  bool threw = false;
  try { interpolateAt(std::vector<std::pair<double, double>>{}, 1.0); } catch (const std::invalid_argument&) { threw = true; }
  CHECK(threw);
  // lerp extrapolates, t outside [0,1]
  CHECK_NEAR(lerp(1.0, 2.0, -1.0), 0.0, 1e-12);
  // larger trajectory: binary search must pick the right bracket
  std::vector<std::pair<double, Eigen::Vector3d>> line;
  for (int i = 0; i < 1000; ++i) line.push_back({0.01 * i, Eigen::Vector3d(i, 2.0 * i, -i)});
  CHECK_MAT_NEAR(interpolateAt(line, 5.005), Eigen::Vector3d(500.5, 1001, -500.5), 1e-6);
  CHECK_MAT_NEAR(interpolateAt(traj, 10.05), Eigen::Vector3d(0.5, 0, 0), 1e-9);
#endif
  return TEST_SUMMARY();
}
