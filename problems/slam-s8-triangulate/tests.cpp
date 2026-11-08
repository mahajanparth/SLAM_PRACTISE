// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <random>

static Eigen::Isometry3d camAt(const Eigen::Vector3d& C, const Eigen::Matrix3d& R_cw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R_cw;
  T.translation() = -R_cw * C;
  return T;
}
static Eigen::Vector2d obs(const Eigen::Isometry3d& T, const Eigen::Vector3d& X) {
  Eigen::Vector3d c = T * X;
  return c.head<2>() / c.z();
}

int main() {
  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T2 = camAt({1, 0.1, 0}, Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()).toRotationMatrix());
  Eigen::Vector3d X(0.5, -0.3, 5);

  Eigen::Vector3d Xd = triangulateDLT(T1, obs(T1, X), T2, obs(T2, X));
  CHECK_MAT_NEAR(Xd, X, 1e-6);
  Eigen::Vector3d Xm = triangulateMidpoint(T1, obs(T1, X), T2, obs(T2, X));
  CHECK_MAT_NEAR(Xm, X, 1e-6);
  CHECK(cheiralityOK(T1, T2, X));
  CHECK(!cheiralityOK(T1, T2, Eigen::Vector3d(0, 0, -3)));
  // parallax: centres (0,0,0) and (1,0.1,0)
  {
    Eigen::Vector3d a = X, b = X - Eigen::Vector3d(1, 0.1, 0);
    CHECK_NEAR(parallaxAngle(T1, T2, X), std::acos(a.normalized().dot(b.normalized())), 1e-9);
  }

#ifdef HIDDEN
  // general poses, both cameras rotated and translated
  {
    Eigen::Isometry3d A = camAt({-2, 1, -1}, Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix());
    Eigen::Isometry3d B = camAt({2, -1, -0.5}, Eigen::AngleAxisd(-0.4, Eigen::Vector3d(0, 1, 0.2).normalized()).toRotationMatrix());
    Eigen::Vector3d P(-0.7, 0.4, 6);
    CHECK_MAT_NEAR(triangulateDLT(A, obs(A, P), B, obs(B, P)), P, 1e-6);
    CHECK_MAT_NEAR(triangulateMidpoint(A, obs(A, P), B, obs(B, P)), P, 1e-6);
  }
  // noisy observations: both methods close to truth and to each other
  {
    std::mt19937 g(3);
    std::normal_distribution<double> n(0, 1e-3);
    Eigen::Vector2d o1 = obs(T1, X) + Eigen::Vector2d(n(g), n(g));
    Eigen::Vector2d o2 = obs(T2, X) + Eigen::Vector2d(n(g), n(g));
    Eigen::Vector3d a = triangulateDLT(T1, o1, T2, o2), b = triangulateMidpoint(T1, o1, T2, o2);
    CHECK((a - X).norm() < 0.1);
    CHECK((a - b).norm() < 0.05);
  }
  // point behind both cameras: DLT still "solves" it, cheirality must reject it
  {
    Eigen::Isometry3d C2 = camAt({1, 0, 0}, Eigen::Matrix3d::Identity());
    Eigen::Vector3d behind(0.3, 0.2, -4);
    Eigen::Vector3d r = triangulateDLT(T1, obs(T1, behind), C2, obs(C2, behind));
    CHECK_MAT_NEAR(r, behind, 1e-6);
    CHECK(!cheiralityOK(T1, C2, r));
    // in front of one, behind the other
    Eigen::Isometry3d flip = camAt({0, 0, 10}, Eigen::Matrix3d::Identity());
    CHECK(!cheiralityOK(T1, flip, Eigen::Vector3d(0, 0, 5)));
  }
  // small baseline: tiny parallax flagged; noise-free still reasonable
  {
    Eigen::Isometry3d S = camAt({0.01, 0, 0}, Eigen::Matrix3d::Identity());
    Eigen::Vector3d far(0, 0, 20);
    double par = parallaxAngle(T1, S, far);
    CHECK(par > 0 && par < 1.0 * M_PI / 180.0);
    CHECK_NEAR(par, std::atan2(0.01, 20), 1e-9);
    CHECK((triangulateDLT(T1, obs(T1, far), S, obs(S, far)) - far).norm() < 1e-3);
  }
#endif
  return TEST_SUMMARY();
}
