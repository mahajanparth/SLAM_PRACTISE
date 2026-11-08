// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static const double PI = 3.14159265358979323846;

int main() {
  std::vector<StampedPose> traj = {
      {0.0, {0, 0, 0}}, {1.0, {1, 0, PI / 2}}, {2.0, {1, 1, PI / 2}}, {3.0, {1, 1, PI - 0.1}}, {4.0, {1, 1, -PI + 0.1}}};

  CHECK_NEAR(wrapAngle(3 * PI / 2), -PI / 2, 1e-12);

  auto n = nearestIndex(traj, 1.3, 0.5);
  CHECK(n.has_value() && *n == 1);
  CHECK(!nearestIndex(traj, 5.0, 0.5).has_value());

  auto p = poseAt(traj, 0.5);
  CHECK(p.has_value());
  if (p) { CHECK_NEAR(p->x, 0.5, 1e-12); CHECK_NEAR(p->theta, PI / 4, 1e-12); }
  CHECK(!poseAt(traj, -0.1).has_value());

  auto w = poseAt(traj, 3.5);  // crosses ±pi: must go through pi, not through 0
  CHECK(w.has_value());
  if (w) CHECK_NEAR(std::cos(w->theta), -1.0, 1e-9);

  Pose2 r = relative({1, 1, PI / 2}, {1, 3, PI / 2});  // 2 m straight ahead
  CHECK_NEAR(r.x, 2.0, 1e-12);
  CHECK_NEAR(r.y, 0.0, 1e-12);

#ifdef HIDDEN
  CHECK_NEAR(wrapAngle(PI), PI, 1e-12);
  CHECK_NEAR(wrapAngle(-PI), PI, 1e-12);
  CHECK_NEAR(wrapAngle(0.3 + 8 * PI), 0.3, 1e-9);
  CHECK_NEAR(wrapAngle(-0.3 - 6 * PI), -0.3, 1e-9);
  // nearest: ties -> earlier; boundaries; empty
  auto tie = nearestIndex(traj, 1.5, 1.0);
  CHECK(tie && *tie == 1);
  auto first = nearestIndex(traj, -0.2, 0.25);
  CHECK(first && *first == 0);
  auto last = nearestIndex(traj, 4.2, 0.25);
  CHECK(last && *last == 4);
  CHECK(!nearestIndex({}, 1.0, 10.0).has_value());
  CHECK(!nearestIndex(traj, 1.5, 0.4).has_value());
  // exact stamps, boundaries inclusive, single sample
  auto e = poseAt(traj, 4.0);
  CHECK(e && std::abs(e->theta - (-PI + 0.1)) < 1e-12);
  auto s = poseAt(traj, 0.0);
  CHECK(s && s->x == 0.0);
  std::vector<StampedPose> one = {{5.0, {1, 2, 3}}};
  auto o = poseAt(one, 5.0);
  CHECK(o && o->y == 2.0);
  CHECK(!poseAt(one, 5.1).has_value());
  CHECK(!poseAt({}, 0.0).has_value());
  // interpolated heading is wrapped
  auto mid = poseAt(traj, 3.75);
  CHECK(mid && mid->theta > -PI && mid->theta <= PI);
  // relative: inverse consistency and rotation handling
  Pose2 a{2, -1, 0.7}, b{-3, 4, -2.9};
  Pose2 ab = relative(a, b), ba = relative(b, a);
  // compose a * ab should give b
  double bx = a.x + std::cos(a.theta) * ab.x - std::sin(a.theta) * ab.y;
  double by = a.y + std::sin(a.theta) * ab.x + std::cos(a.theta) * ab.y;
  CHECK(std::abs(bx - b.x) < 1e-9 && std::abs(by - b.y) < 1e-9);
  CHECK_NEAR(wrapAngle(ab.theta + ba.theta), 0.0, 1e-12);
  CHECK(ab.theta > -PI && ab.theta <= PI);
  // motionBetween
  auto mv = motionBetween(traj, 1.0, 2.0);  // drove 1 m forward (facing +y)
  CHECK(mv && std::abs(mv->x - 1.0) < 1e-12 && std::abs(mv->y) < 1e-12 && std::abs(mv->theta) < 1e-12);
  auto mt = motionBetween(traj, 3.0, 4.0);  // pure rotation of +0.2 across the cut
  CHECK(mt && std::abs(mt->theta - 0.2) < 1e-9);
  CHECK(!motionBetween(traj, 3.0, 4.5).has_value());
  // large trajectory: O(log n) lookups
  std::vector<StampedPose> big;
  for (int i = 0; i < 200000; ++i) big.push_back({i * 0.02, {i * 0.01, 0, wrapAngle(i * 0.001)}});
  bool ok = true;
  for (int k = 0; k < 20000 && ok; ++k) {
    double t = k * 0.1999 + 0.005;
    auto q = poseAt(big, t);
    ok = q && std::abs(q->x - t * 0.5) < 1e-9;
    auto ni = nearestIndex(big, t, 0.0101);
    ok = ok && ni && std::abs(big[*ni].t - t) <= 0.01 + 1e-12;
  }
  CHECK(ok);
#endif
  return TEST_SUMMARY();
}
