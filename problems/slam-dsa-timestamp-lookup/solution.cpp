#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

struct StampedPose {
  double t;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
};

namespace {
// First sample with stamp >= t, O(log n).
std::vector<StampedPose>::const_iterator firstNotBefore(const std::vector<StampedPose>& traj, double t) {
  return std::lower_bound(traj.begin(), traj.end(), t,
                          [](const StampedPose& s, double v) { return s.t < v; });
}
}  // namespace

std::optional<StampedPose> nearestPose(const std::vector<StampedPose>& traj, double t, double max_gap) {
  if (traj.empty()) return std::nullopt;
  auto it = firstNotBefore(traj, t);
  if (it == traj.end()) it = traj.end() - 1;              // past the end: last sample
  else if (it != traj.begin() && (t - (it - 1)->t) <= (it->t - t)) --it;  // tie -> earlier
  if (std::abs(it->t - t) > max_gap) return std::nullopt;
  return *it;
}

std::optional<StampedPose> interpolatePose(const std::vector<StampedPose>& traj, double t, double max_gap) {
  if (traj.empty() || t < traj.front().t || t > traj.back().t) return std::nullopt;
  auto it = firstNotBefore(traj, t);  // valid: t <= back().t
  if (it->t == t) return *it;          // exact hit (also covers the single-sample case)
  const StampedPose& a = *(it - 1);    // it != begin() because t > front().t
  const StampedPose& b = *it;
  if (b.t - a.t > max_gap) return std::nullopt;  // dropped messages: don't bridge the hole
  const double alpha = (t - a.t) / (b.t - a.t);
  StampedPose out;
  out.t = t;
  out.p = (1.0 - alpha) * a.p + alpha * b.p;
  out.q = a.q.slerp(alpha, b.q).normalized();  // Eigen's slerp takes the shortest arc (handles q vs -q)
  return out;
}
