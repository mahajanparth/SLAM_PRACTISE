// Test-data helpers the "interviewer" uses to generate inputs. Not part of any answer.
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace cu {

using Cloud = std::vector<Eigen::Vector3d>;

inline std::mt19937& rng(unsigned seed = 42) {
  static std::mt19937 g(seed);
  return g;
}

inline Cloud uniformCube(int n, double half = 1.0) {
  std::uniform_real_distribution<double> u(-half, half);
  Cloud c(n);
  for (auto& p : c) p = {u(rng()), u(rng()), u(rng())};
  return c;
}

// Points on plane n.x + d = 0 (n normalized internally), spread over `extent`.
inline Cloud planePoints(int n, Eigen::Vector3d normal, double d, double extent = 5.0,
                         double noise = 0.0) {
  normal.normalize();
  Eigen::Vector3d a = normal.unitOrthogonal(), b = normal.cross(a);
  std::uniform_real_distribution<double> u(-extent, extent);
  std::normal_distribution<double> g(0.0, noise > 0 ? noise : 1e-12);
  Cloud c(n);
  for (auto& p : c) p = -d * normal + u(rng()) * a + u(rng()) * b + g(rng()) * normal;
  return c;
}

inline Cloud spherePoints(int n, const Eigen::Vector3d& center, double r) {
  std::normal_distribution<double> g(0, 1);
  Cloud c(n);
  for (auto& p : c) p = center + r * Eigen::Vector3d(g(rng()), g(rng()), g(rng())).normalized();
  return c;
}

inline void addNoise(Cloud& c, double sigma) {
  std::normal_distribution<double> g(0, sigma);
  for (auto& p : c) p += Eigen::Vector3d(g(rng()), g(rng()), g(rng()));
}

inline Eigen::Isometry3d randomPose(double max_angle = M_PI, double max_trans = 1.0) {
  std::uniform_real_distribution<double> u(-1, 1);
  Eigen::Vector3d axis(u(rng()), u(rng()), u(rng()));
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(max_angle * std::abs(u(rng())), axis.normalized()).toRotationMatrix();
  T.translation() = max_trans * Eigen::Vector3d(u(rng()), u(rng()), u(rng()));
  return T;
}

inline Cloud transform(const Cloud& c, const Eigen::Isometry3d& T) {
  Cloud out(c.size());
  for (size_t i = 0; i < c.size(); ++i) out[i] = T * c[i];
  return out;
}

inline Eigen::Matrix3Xd toMatrix(const Cloud& c) {
  Eigen::Matrix3Xd M(3, c.size());
  for (size_t i = 0; i < c.size(); ++i) M.col(i) = c[i];
  return M;
}

struct Timer {
  std::string name;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  explicit Timer(std::string n) : name(std::move(n)) {}
  ~Timer() {
    auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[timer] " << name << ": " << ms << " ms\n";
  }
};

}  // namespace cu
