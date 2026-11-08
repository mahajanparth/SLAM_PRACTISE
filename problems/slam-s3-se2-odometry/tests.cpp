// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static double angDiff(double a, double b) { return std::abs(std::remainder(a - b, 2 * M_PI)); }

int main() {
  CHECK_NEAR(wrapAngle(1.5 * M_PI), -0.5 * M_PI, 1e-12);
  CHECK_NEAR(wrapAngle(-2.5 * M_PI), -0.5 * M_PI, 1e-12);
  CHECK_NEAR(wrapAngle(0.3), 0.3, 1e-15);

  // compose: a at (1,1) facing +y, b = 2 m forward + turn 90 deg -> (1,3) facing -x
  Pose2 c = compose({1, 1, M_PI / 2}, {2, 0, M_PI / 2});
  CHECK_NEAR(c.x, 1.0, 1e-12);
  CHECK_NEAR(c.y, 3.0, 1e-12);
  CHECK(angDiff(c.theta, M_PI) < 1e-12);

  // one long sample: quarter circle, v = 1, omega = pi/2, dt = 1 -> radius 2/pi
  Pose2 q = integrateOdometry({0, 0, 0}, {{1.0, M_PI / 2, 1.0}});
  CHECK_NEAR(q.x, 2.0 / M_PI, 1e-12);
  CHECK_NEAR(q.y, 2.0 / M_PI, 1e-12);
  CHECK_NEAR(q.theta, M_PI / 2, 1e-12);

  // straight line
  Pose2 s = integrateOdometry({1, 2, M_PI / 4}, {{2.0, 0.0, 1.5}});
  CHECK_NEAR(s.x, 1 + 3 * std::cos(M_PI / 4), 1e-12);
  CHECK_NEAR(s.y, 2 + 3 * std::sin(M_PI / 4), 1e-12);

#ifdef HIDDEN
  // tiny omega: no NaN, ~straight
  Pose2 t = integrateOdometry({0, 0, 0}, {{1.0, 1e-12, 2.0}});
  CHECK(std::isfinite(t.x) && std::isfinite(t.y));
  CHECK_NEAR(t.x, 2.0, 1e-9);
  CHECK_NEAR(t.y, 0.0, 1e-9);
  // pure rotation in place, many samples -> heading wraps, position unchanged
  std::vector<OdomSample> spin(100, OdomSample{0.0, 1.0, 0.1});  // 10 rad
  Pose2 sp = integrateOdometry({3, 4, 0}, spin);
  CHECK_NEAR(sp.x, 3.0, 1e-12);
  CHECK_NEAR(sp.y, 4.0, 1e-12);
  CHECK(sp.theta >= -M_PI - 1e-12 && sp.theta <= M_PI + 1e-12);
  CHECK(angDiff(sp.theta, 10.0) < 1e-9);
  // full circle in 1000 steps returns to the start (exact arcs -> no drift)
  std::vector<OdomSample> circ(1000, OdomSample{2.0, 2 * M_PI / 10.0, 0.01});  // T = 10 s
  Pose2 cc = integrateOdometry({-1, 5, 2.0}, circ);
  CHECK_NEAR(cc.x, -1.0, 1e-9);
  CHECK_NEAR(cc.y, 5.0, 1e-9);
  CHECK(angDiff(cc.theta, 2.0) < 1e-9);
  // empty samples
  Pose2 e = integrateOdometry({1, 2, 0.5}, {});
  CHECK_NEAR(e.x, 1.0, 0.0);
  CHECK_NEAR(e.theta, 0.5, 1e-15);
  // reversing (negative v) with negative omega: half circle
  Pose2 h = integrateOdometry({0, 0, 0}, {{-1.0, -1.0, M_PI}});
  CHECK_NEAR(h.x, 0.0, 1e-12);
  CHECK_NEAR(h.y, 2.0, 1e-12);
  // compose with heading wrap and identity
  Pose2 w = compose({0, 0, 3.0}, {0, 0, 3.0});
  CHECK_NEAR(w.theta, wrapAngle(6.0), 1e-12);
  Pose2 idc = compose({1, 2, 0.3}, {0, 0, 0});
  CHECK_NEAR(idc.x, 1.0, 1e-15);
  CHECK_NEAR(idc.y, 2.0, 1e-15);
  // integrating = composing the per-sample arc motions
  Pose2 seq = integrateOdometry({0, 0, 0}, {{1.0, 0.5, 1.0}, {0.5, -1.0, 2.0}});
  Pose2 m1 = integrateOdometry({0, 0, 0}, {{1.0, 0.5, 1.0}});
  Pose2 m2 = integrateOdometry({0, 0, 0}, {{0.5, -1.0, 2.0}});
  Pose2 cm = compose(m1, m2);
  CHECK_NEAR(seq.x, cm.x, 1e-12);
  CHECK_NEAR(seq.y, cm.y, 1e-12);
#endif
  return TEST_SUMMARY();
}
