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

std::optional<StampedPose> nearestPose(const std::vector<StampedPose>& traj, double t, double max_gap) {
  // TODO
  return std::nullopt;
}

std::optional<StampedPose> interpolatePose(const std::vector<StampedPose>& traj, double t, double max_gap) {
  // TODO
  return std::nullopt;
}
