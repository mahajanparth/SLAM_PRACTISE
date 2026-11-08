// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static const double PI = 3.14159265358979323846;

int main() {
  Vec3d a{{1, 2, 3}}, b{{4, 6, 3}};
  Vec3d d = b - a;
  CHECK_NEAR(d[0], 3.0, 1e-12);
  CHECK_NEAR(d.norm(), 5.0, 1e-12);
  CHECK_NEAR(a.dot(b), 4 + 12 + 9, 1e-12);
  Vec3d s = 2.0 * a + a * 0.5;
  CHECK_NEAR(s[2], 7.5, 1e-12);

  CHECK_NEAR(interpolate(10.0, 20.0, 0.25), 12.5, 1e-12);
  Vec3d m = interpolate(a, b, 0.5);
  CHECK_NEAR(m[1], 4.0, 1e-12);

  // 170deg -> -170deg: shortest arc goes through 180, not through 0
  Angle h = interpolate(Angle{170 * PI / 180}, Angle{-170 * PI / 180}, 0.5);
  CHECK_NEAR(std::cos(h.rad), -1.0, 1e-9);
  CHECK(h.rad > -PI && h.rad <= PI);

  std::vector<double> stamps = {0.0, 1.0, 3.0};
  std::vector<double> xs = {0.0, 10.0, 30.0};
  CHECK_NEAR(sampleAt(stamps, xs, 2.0), 20.0, 1e-12);

#ifdef HIDDEN
  // integer vectors
  Vec<int, 2> vi{{3, 4}};
  CHECK_EQ(vi.dot(vi), 25);
  CHECK_NEAR(vi.norm(), 5.0, 1e-12);
  CHECK_EQ((3 * vi)[1], 12);
  // clamping and empty
  CHECK_NEAR(sampleAt(stamps, xs, -5.0), 0.0, 1e-12);
  CHECK_NEAR(sampleAt(stamps, xs, 99.0), 30.0, 1e-12);
  CHECK_NEAR(sampleAt(stamps, xs, 1.0), 10.0, 1e-12);
  CHECK_NEAR(sampleAt<double>({}, {}, 1.0), 0.0, 1e-15);
  CHECK_NEAR(sampleAt<double>({5.0}, {7.0}, 1.0), 7.0, 1e-15);
  // generic code must pick the Angle specialization
  std::vector<Angle> yaw = {Angle{3.0}, Angle{-3.0}};
  Angle y = sampleAt(std::vector<double>{0.0, 1.0}, yaw, 0.5);
  CHECK_NEAR(std::cos(y.rad), -1.0, 1e-6);
  // endpoints, wrapping of unwrapped inputs, opposite angles go CCW
  Angle e0 = interpolate(Angle{0.3}, Angle{1.0}, 0.0);
  CHECK_NEAR(e0.rad, 0.3, 1e-12);
  Angle e1 = interpolate(Angle{0.3}, Angle{1.0 + 4 * PI}, 1.0);
  CHECK_NEAR(e1.rad, 1.0, 1e-9);
  Angle q = interpolate(Angle{0.0}, Angle{PI}, 0.5);
  CHECK_NEAR(q.rad, PI / 2, 1e-9);
  Angle w = interpolate(Angle{-PI + 0.1}, Angle{PI - 0.1}, 0.5);  // crosses the cut at ±pi
  CHECK_NEAR(std::cos(w.rad), -1.0, 1e-9);
  Angle big = interpolate(Angle{7 * PI}, Angle{7 * PI}, 0.3);  // input outside range -> wrapped output
  CHECK(big.rad > -PI && big.rad <= PI);
  CHECK_NEAR(std::cos(big.rad), -1.0, 1e-9);
  // Vec time series
  std::vector<Vec3d> pos = {Vec3d{{0, 0, 0}}, Vec3d{{1, 2, 3}}, Vec3d{{1, 2, 13}}};
  Vec3d p = sampleAt(stamps, pos, 2.5);
  CHECK_NEAR(p[2], 10.5, 1e-12);
  CHECK_NEAR(p[0], 1.0, 1e-12);
#endif
  return TEST_SUMMARY();
}
