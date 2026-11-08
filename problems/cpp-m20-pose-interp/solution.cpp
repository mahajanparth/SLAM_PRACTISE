#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

struct Pose2 { double x, y, theta; };
struct StampedPose { double t; Pose2 pose; };

double wrapAngle(double a) {
  constexpr double kPi = 3.14159265358979323846;
  a = std::fmod(a + kPi, 2.0 * kPi);  // (-2pi, 2pi)
  if (a <= 0.0) a += 2.0 * kPi;       // (0, 2pi]
  return a - kPi;                     // (-pi, pi]
}

static std::vector<StampedPose>::const_iterator firstNotBefore(const std::vector<StampedPose>& traj, double t) {
  return std::lower_bound(traj.begin(), traj.end(), t,
                          [](const StampedPose& s, double x) { return s.t < x; });
}

std::optional<std::size_t> nearestIndex(const std::vector<StampedPose>& traj, double t, double max_dt) {
  if (traj.empty()) return std::nullopt;
  auto it = firstNotBefore(traj, t);
  std::size_t best;
  if (it == traj.begin()) {
    best = 0;
  } else if (it == traj.end()) {
    best = traj.size() - 1;
  } else {
    const std::size_t hi = static_cast<std::size_t>(it - traj.begin()), lo = hi - 1;
    best = (t - traj[lo].t <= traj[hi].t - t) ? lo : hi;  // <= : ties go to the earlier sample
  }
  if (std::abs(traj[best].t - t) > max_dt) return std::nullopt;
  return best;
}

std::optional<Pose2> poseAt(const std::vector<StampedPose>& traj, double t) {
  if (traj.empty() || t < traj.front().t || t > traj.back().t) return std::nullopt;  // never extrapolate
  auto it = firstNotBefore(traj, t);
  if (it->t == t) return it->pose;  // exact sample (also covers a single-sample trajectory)
  const StampedPose& b = *it;
  const StampedPose& a = *(it - 1);  // it != begin, since t > front.t
  const double alpha = (t - a.t) / (b.t - a.t);
  return Pose2{a.pose.x + alpha * (b.pose.x - a.pose.x),
               a.pose.y + alpha * (b.pose.y - a.pose.y),
               wrapAngle(a.pose.theta + alpha * wrapAngle(b.pose.theta - a.pose.theta))};  // shortest arc
}

Pose2 relative(const Pose2& a, const Pose2& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  // R(-theta_a) * (p_b - p_a)
  return {c * dx + s * dy, -s * dx + c * dy, wrapAngle(b.theta - a.theta)};
}

std::optional<Pose2> motionBetween(const std::vector<StampedPose>& traj, double t0, double t1) {
  const auto p0 = poseAt(traj, t0);
  const auto p1 = poseAt(traj, t1);
  if (!p0 || !p1) return std::nullopt;
  return relative(*p0, *p1);
}
