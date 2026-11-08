// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static const Eigen::Vector3d G(0, 0, -9.81);

static std::vector<ImuSample> constant(int n, double dt, Eigen::Vector3d w, Eigen::Vector3d a) {
  return std::vector<ImuSample>(n, ImuSample{dt, w, a});
}

int main() {
  const ImuState s0{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  const Eigen::Vector3d z3 = Eigen::Vector3d::Zero();

  Eigen::Vector3d phi(0.3, -0.2, 0.5);
  CHECK_MAT_NEAR(expSO3(phi), Eigen::AngleAxisd(phi.norm(), phi.normalized()).toRotationMatrix(), 1e-12);

  // level & stationary: accel reads +g
  ImuState a = integrateImu(s0, constant(200, 0.005, z3, {0, 0, 9.81}), z3, z3, G);
  CHECK_MAT_NEAR(a.p, z3, 1e-9);
  CHECK_MAT_NEAR(a.v, z3, 1e-9);

  // constant yaw rate 0.5 rad/s for 2 s
  ImuState b = integrateImu(s0, constant(400, 0.005, {0, 0, 0.5}, {0, 0, 9.81}), z3, z3, G);
  CHECK_MAT_NEAR(b.R, Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(), 1e-9);

  // constant forward accel 1 m/s^2 for 2 s from v0 = (0,1,0)
  ImuState s1 = s0; s1.v = Eigen::Vector3d(0, 1, 0);
  ImuState c = integrateImu(s1, constant(400, 0.005, z3, {1, 0, 9.81}), z3, z3, G);
  CHECK_MAT_NEAR(c.v, Eigen::Vector3d(2, 1, 0), 1e-9);
  CHECK_MAT_NEAR(c.p, Eigen::Vector3d(2, 2, 0), 1e-9);

#ifdef HIDDEN
  // small-angle exp: no NaN, ~identity
  Eigen::Matrix3d Rs = expSO3(Eigen::Vector3d(1e-12, 0, -1e-12));
  CHECK(Rs.allFinite());
  CHECK_MAT_NEAR(Rs, Eigen::Matrix3d::Identity(), 1e-11);
  CHECK_MAT_NEAR(expSO3(z3), Eigen::Matrix3d::Identity(), 0.0);
  // near-pi
  Eigen::Vector3d pi_axis = Eigen::Vector3d(1, 1, 0).normalized() * (M_PI - 1e-6);
  CHECK_MAT_NEAR(expSO3(pi_axis), Eigen::AngleAxisd(pi_axis.norm(), pi_axis.normalized()).toRotationMatrix(), 1e-9);

  // empty input: unchanged
  ImuState e = integrateImu(s1, {}, z3, z3, G);
  CHECK_MAT_NEAR(e.v, s1.v, 0.0);

  // tilted & stationary with biases: accel reads R^T(-g) + b_a, gyro reads b_g
  {
    Eigen::Matrix3d R0 = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, -2, 0.5).normalized()).toRotationMatrix();
    Eigen::Vector3d bg(0.01, -0.02, 0.005), ba(0.1, -0.05, 0.2);
    ImuState t{R0, z3, Eigen::Vector3d(1, 2, 3)};
    ImuState r = integrateImu(t, constant(400, 0.005, bg, R0.transpose() * -G + ba), bg, ba, G);
    CHECK_MAT_NEAR(r.p, Eigen::Vector3d(1, 2, 3), 1e-9);
    CHECK_MAT_NEAR(r.R, R0, 1e-9);
  }

  // order matters: rotate about z while accelerating in body x -> uses OLD R each step
  {
    std::vector<ImuSample> two = {{1.0, {0, 0, M_PI / 2}, {1, 0, 9.81}}, {1.0, z3, {1, 0, 9.81}}};
    ImuState r = integrateImu(s0, two, z3, z3, G);
    // step1: a_w=(1,0,0): p=(0.5,0,0) v=(1,0,0); R->Rz(90). step2: a_w=(0,1,0): p=(1.5,0.5,0) v=(1,1,0)
    CHECK_MAT_NEAR(r.p, Eigen::Vector3d(1.5, 0.5, 0), 1e-9);
    CHECK_MAT_NEAR(r.v, Eigen::Vector3d(1, 1, 0), 1e-9);
  }

  // long run: R stays a rotation; split integration equals one-shot
  {
    auto many = constant(10000, 0.005, {0.3, -0.7, 1.1}, {0.2, 0.1, 9.7});
    ImuState L = integrateImu(s0, many, z3, z3, G);
    CHECK_MAT_NEAR(L.R.transpose() * L.R, Eigen::Matrix3d::Identity(), 1e-9);
    CHECK_NEAR(L.R.determinant(), 1.0, 1e-9);
    std::vector<ImuSample> h1(many.begin(), many.begin() + 5000), h2(many.begin() + 5000, many.end());
    ImuState L2 = integrateImu(integrateImu(s0, h1, z3, z3, G), h2, z3, z3, G);
    CHECK_MAT_NEAR(L2.p, L.p, 1e-6);
  }
#endif
  return TEST_SUMMARY();
}
