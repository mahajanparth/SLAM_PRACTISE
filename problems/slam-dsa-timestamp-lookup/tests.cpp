// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static StampedPose mk(double t, double x, double yaw) {
  return {t, Eigen::Vector3d(x, 2 * x, 0), Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))};
}
static double yawOf(const Eigen::Quaterniond& q) {
  Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

int main() {
  std::vector<StampedPose> traj;
  for (int i = 0; i <= 100; ++i) traj.push_back(mk(i * 0.01, i * 0.1, i * 0.01));  // 100 Hz

  auto n = nearestPose(traj, 0.123, 0.02);
  CHECK(n.has_value());
  if (n) CHECK_NEAR(n->t, 0.12, 1e-12);
  CHECK(!nearestPose(traj, 5.0, 0.02).has_value());  // too far from any sample

  auto ip = interpolatePose(traj, 0.125, 0.02);
  CHECK(ip.has_value());
  if (ip) {
    CHECK_NEAR(ip->t, 0.125, 1e-12);
    CHECK_NEAR(ip->p.x(), 1.25, 1e-9);
    CHECK_NEAR(yawOf(ip->q), 0.125, 1e-9);
  }
  CHECK(!interpolatePose(traj, -0.001, 0.02).has_value());  // no extrapolation

#ifdef HIDDEN
  std::vector<StampedPose> empty;
  CHECK(!nearestPose(empty, 0.0, 1.0).has_value());
  CHECK(!interpolatePose(empty, 0.0, 1.0).has_value());
  // before the first / after the last sample, within the gap: nearest clamps
  auto f = nearestPose(traj, -0.004, 0.01);
  CHECK(f.has_value() && std::abs(f->t - 0.0) < 1e-12);
  auto l = nearestPose(traj, 1.004, 0.01);
  CHECK(l.has_value() && std::abs(l->t - 1.0) < 1e-12);
  // tie -> earlier
  auto tie = nearestPose(traj, 0.015, 0.02);
  CHECK(tie.has_value() && std::abs(tie->t - 0.01) < 1e-9);
  // exact hit & single sample
  std::vector<StampedPose> one = {mk(2.0, 1.0, 0.3)};
  auto e = interpolatePose(one, 2.0, 0.1);
  CHECK(e.has_value() && std::abs(e->p.x() - 1.0) < 1e-12);
  CHECK(!interpolatePose(one, 2.05, 0.1).has_value());
  // gap (dropped messages) is rejected
  std::vector<StampedPose> gap = {mk(0.0, 0, 0), mk(0.5, 5, 0.5)};
  CHECK(!interpolatePose(gap, 0.25, 0.1).has_value());
  CHECK(interpolatePose(gap, 0.25, 1.0).has_value());
  // quaternion sign flip: q and -q describe the same rotation; result must be the midpoint yaw
  std::vector<StampedPose> flip = {mk(0.0, 0, 3.0), mk(0.1, 1, -3.0)};  // crosses +-pi
  flip[1].q.coeffs() *= -1.0;
  auto m = interpolatePose(flip, 0.05, 1.0);
  CHECK(m.has_value());
  if (m) CHECK_NEAR(std::abs(yawOf(m->q)), M_PI, 1e-6);
  // large n, many queries (must be log n)
  std::vector<StampedPose> big;
  big.reserve(100000);
  for (int i = 0; i < 100000; ++i) big.push_back(mk(i * 0.01, i * 0.01, 0));
  double acc = 0;
  for (int k = 0; k < 20000; ++k) {
    auto r = interpolatePose(big, k * 0.0497 + 0.003, 0.02);
    if (r) acc += r->p.x() - (k * 0.0497 + 0.003);
  }
  CHECK_NEAR(acc, 0.0, 1e-6);
#endif
  return TEST_SUMMARY();
}
