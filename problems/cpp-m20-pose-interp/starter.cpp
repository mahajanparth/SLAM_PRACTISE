#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

struct Pose2 { double x, y, theta; };
struct StampedPose { double t; Pose2 pose; };

double wrapAngle(double a) {
  // TODO: (-pi, pi]
  return a;
}

std::optional<std::size_t> nearestIndex(const std::vector<StampedPose>& traj, double t, double max_dt) {
  // TODO
  return std::nullopt;
}

std::optional<Pose2> poseAt(const std::vector<StampedPose>& traj, double t) {
  // TODO
  return std::nullopt;
}

Pose2 relative(const Pose2& a, const Pose2& b) {
  // TODO: a^{-1} * b
  return {0, 0, 0};
}

std::optional<Pose2> motionBetween(const std::vector<StampedPose>& traj, double t0, double t1) {
  // TODO
  return std::nullopt;
}
