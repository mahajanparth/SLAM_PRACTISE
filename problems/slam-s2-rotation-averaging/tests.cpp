// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

static double angle(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q) {
  const Eigen::Quaterniond d = p.conjugate() * q;
  return 2.0 * std::atan2(d.vec().norm(), std::abs(d.w()));
}
static Eigen::Quaterniond qexp(const Eigen::Vector3d& w) {
  const double th = w.norm();
  return th < 1e-15 ? Eigen::Quaterniond::Identity() : Eigen::Quaterniond(Eigen::AngleAxisd(th, w / th));
}
static Eigen::Quaterniond flip(const Eigen::Quaterniond& q) { return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z()); }

int main() {
  const Eigen::Quaterniond q0 = qexp(Eigen::Vector3d(0.4, -1.2, 2.0));  // far from identity
  // symmetric pairs around q0, half of them sign-flipped -> naive averaging breaks
  std::vector<Eigen::Quaterniond> qs;
  std::vector<double> w;
  const Eigen::Vector3d ds[] = {{0.05, 0, 0}, {0, 0.08, 0}, {0, 0, 0.03}, {0.02, -0.04, 0.06}};
  for (int i = 0; i < 4; ++i) {
    qs.push_back(q0 * qexp(ds[i]));
    qs.push_back(flip(q0 * qexp(-ds[i])));
    w.push_back(1.0); w.push_back(1.0);
  }
  Eigen::Quaterniond m = averageQuaternions(qs, w);
  CHECK_NEAR(m.norm(), 1.0, 1e-9);
  CHECK(angle(m, q0) < 1e-9);

  // noisy samples
  std::normal_distribution<double> g(0.0, 0.05);
  std::vector<Eigen::Quaterniond> noisy;
  for (int i = 0; i < 200; ++i) {
    Eigen::Quaterniond q = q0 * qexp(Eigen::Vector3d(g(cu::rng()), g(cu::rng()), g(cu::rng())));
    noisy.push_back(i % 3 == 0 ? flip(q) : q);
  }
  Eigen::Quaterniond mn = averageQuaternions(noisy, std::vector<double>(noisy.size(), 1.0));
  CHECK(angle(mn, q0) < 0.02);

#ifdef HIDDEN
  // empty and single
  Eigen::Quaterniond e = averageQuaternions({}, {});
  CHECK(angle(e, Eigen::Quaterniond::Identity()) < 1e-12);
  CHECK(angle(averageQuaternions({flip(q0)}, {2.0}), q0) < 1e-9);
  // q and -q only
  CHECK(angle(averageQuaternions({q0, flip(q0)}, {1.0, 1.0}), q0) < 1e-9);
  // large symmetric spread (+-60 deg about 3 axes)
  std::vector<Eigen::Quaterniond> wide;
  for (int a = 0; a < 3; ++a) {
    Eigen::Vector3d d = Eigen::Vector3d::Zero(); d(a) = M_PI / 3;
    wide.push_back(q0 * qexp(d));
    wide.push_back(flip(q0 * qexp(-d)));
  }
  CHECK(angle(averageQuaternions(wide, std::vector<double>(6, 1.0)), q0) < 1e-9);
  // weights: 1:3 between two rotations about the same axis -> 3/4 of the way (chordal ~ geodesic here)
  Eigen::Quaterniond qa = q0, qb = q0 * qexp(Eigen::Vector3d(0, 0, 0.2));
  Eigen::Quaterniond mw = averageQuaternions({qa, flip(qb)}, {1.0, 3.0});
  CHECK(angle(mw, q0 * qexp(Eigen::Vector3d(0, 0, 0.15))) < 1e-3);
  // first sample is far from the rest: sign alignment to qs[0] must not break things
  std::vector<Eigen::Quaterniond> lop = {q0 * qexp(Eigen::Vector3d(0.9, 0, 0))};
  std::vector<double> lw = {0.01};
  for (int i = 0; i < 10; ++i) {
    lop.push_back(i % 2 ? flip(q0 * qexp(Eigen::Vector3d(0, 0.01 * i, 0))) : q0 * qexp(Eigen::Vector3d(0, -0.01 * i, 0)));
    lw.push_back(1.0);
  }
  CHECK(angle(averageQuaternions(lop, lw), q0) < 0.03);
#endif
  return TEST_SUMMARY();
}
