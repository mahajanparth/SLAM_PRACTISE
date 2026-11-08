// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <random>

static Eigen::Vector2d measure(const Eigen::Vector3d& x, const Eigen::Vector2d& m) {
  Eigen::Vector2d d = m - x.head<2>();
  return {d.norm(), std::atan2(std::sin(std::atan2(d.y(), d.x()) - x(2)),
                               std::cos(std::atan2(d.y(), d.x()) - x(2)))};
}

static bool isSymPD(const Eigen::Matrix3d& P) {
  if ((P - P.transpose()).cwiseAbs().maxCoeff() > 1e-9) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P);
  return es.eigenvalues().minCoeff() > 0;
}

int main() {
  const Eigen::Matrix3d Q = Eigen::Vector3d(0.01, 0.01, 0.001).asDiagonal();
  const Eigen::Matrix2d R = Eigen::Vector2d(0.01, 0.0004).asDiagonal();

  CHECK_NEAR(wrapAngle(3 * M_PI / 2), -M_PI / 2, 1e-12);

  // --- predict: mean and covariance
  EkfState s{Eigen::Vector3d(1, 2, M_PI / 2), Eigen::Matrix3d::Identity() * 0.1};
  ekfPredict(s, 1.0, 0.5, 0.2, Q);
  CHECK_MAT_NEAR(s.x, Eigen::Vector3d(1, 2.2, M_PI / 2 + 0.1), 1e-12);
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = -0.2; F(1, 2) = 0.0;
  Eigen::Matrix3d Pexp = F * (Eigen::Matrix3d::Identity() * 0.1) * F.transpose() + Q;
  CHECK_MAT_NEAR(s.P, Pexp, 1e-12);

  // --- update: reference computation
  {
    EkfState u{Eigen::Vector3d(0, 0, 0.1), Eigen::Vector3d(0.2, 0.3, 0.05).asDiagonal()};
    Eigen::Vector2d m(4, 3);
    Eigen::Vector2d z(5.1, std::atan2(3, 4) - 0.05);
    // reference
    Eigen::Vector2d d = m;
    double q = d.squaredNorm(), r = std::sqrt(q);
    Eigen::Matrix<double, 2, 3> H; H << -d.x() / r, -d.y() / r, 0, d.y() / q, -d.x() / q, -1;
    Eigen::Vector2d y = z - Eigen::Vector2d(r, std::atan2(3, 4) - 0.1);
    Eigen::Matrix2d S = H * u.P * H.transpose() + R;
    Eigen::Matrix<double, 3, 2> K = u.P * H.transpose() * S.inverse();
    Eigen::Vector3d xexp = u.x + K * y;
    Eigen::Matrix3d Pexp2 = (Eigen::Matrix3d::Identity() - K * H) * u.P;
    double tr0 = u.P.trace();
    ekfUpdate(u, z, m, R);
    CHECK_MAT_NEAR(u.x, xexp, 1e-9);
    CHECK_MAT_NEAR(u.P, Pexp2, 1e-9);
    CHECK(u.P.trace() < tr0);
  }

#ifdef HIDDEN
  // wrap of predicted heading
  EkfState w{Eigen::Vector3d(0, 0, M_PI - 0.05), Eigen::Matrix3d::Identity() * 0.01};
  ekfPredict(w, 0.0, 1.0, 0.1, Q);
  CHECK_NEAR(w.x(2), -M_PI + 0.05, 1e-9);

  // zero omega, zero v: mean unchanged, P grows by Q
  EkfState zz{Eigen::Vector3d(3, -1, 0.7), Eigen::Matrix3d::Identity() * 0.05};
  ekfPredict(zz, 0.0, 0.0, 0.1, Q);
  CHECK_MAT_NEAR(zz.x, Eigen::Vector3d(3, -1, 0.7), 1e-12);
  CHECK_MAT_NEAR(zz.P, Eigen::Matrix3d::Identity() * 0.05 + Q, 1e-12);

  // bearing innovation near +-pi: landmark directly behind the robot
  {
    Eigen::Vector3d truth(0, 0, 0.02);
    EkfState b{Eigen::Vector3d(0, 0, -0.02), Eigen::Vector3d(0.01, 0.01, 0.01).asDiagonal()};
    Eigen::Vector2d m(-5, 0);            // bearing ~ +-pi
    Eigen::Vector2d z = measure(truth, m);
    ekfUpdate(b, z, m, R);
    CHECK(std::abs(b.x(2) - 0.02) < 0.03);   // moved toward truth, not flung by 2*pi
    CHECK(std::abs(b.x.head<2>().norm()) < 0.2);
  }

  // long run: tracks truth, P stays symmetric PD, theta wrapped
  {
    std::mt19937 g(7);
    std::normal_distribution<double> nr(0, 0.1), nb(0, 0.02);
    std::vector<Eigen::Vector2d> lms = {{5, 5}, {-5, 5}, {-5, -5}, {5, -5}, {0, 8}};
    Eigen::Vector3d truth(2, 0, M_PI / 2);
    EkfState e{truth + Eigen::Vector3d(0.3, -0.2, 0.1), Eigen::Matrix3d::Identity() * 0.5};
    bool pd = true, wrapped = true;
    for (int k = 0; k < 400; ++k) {
      double v = 1.0, om = 0.5, dt = 0.1;
      truth(0) += v * dt * std::cos(truth(2));
      truth(1) += v * dt * std::sin(truth(2));
      truth(2) = std::atan2(std::sin(truth(2) + om * dt), std::cos(truth(2) + om * dt));
      ekfPredict(e, v, om, dt, Q);
      for (const auto& m : lms) {
        Eigen::Vector2d z = measure(truth, m) + Eigen::Vector2d(nr(g), nb(g));
        ekfUpdate(e, z, m, R);
      }
      pd = pd && isSymPD(e.P);
      wrapped = wrapped && e.x(2) > -M_PI - 1e-12 && e.x(2) <= M_PI + 1e-12;
    }
    CHECK(pd);
    CHECK(wrapped);
    CHECK((e.x.head<2>() - truth.head<2>()).norm() < 0.2);
    CHECK(std::abs(wrapAngle(e.x(2) - truth(2))) < 0.05);
  }
#endif
  return TEST_SUMMARY();
}
