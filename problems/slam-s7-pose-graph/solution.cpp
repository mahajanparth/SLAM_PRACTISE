#include <Eigen/Dense>
#include <cmath>
#include <vector>

struct Pose2 { double x = 0, y = 0, theta = 0; };
struct Edge2 {
  int i = 0, j = 0;
  Pose2 z;  // measured x_i^{-1} x_j
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
};

namespace {
double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Eigen::Matrix2d rot(double th) {
  Eigen::Matrix2d R;
  R << std::cos(th), -std::sin(th), std::sin(th), std::cos(th);
  return R;
}
}  // namespace

Eigen::Vector3d edgeError(const Pose2& xi, const Pose2& xj, const Pose2& z) {
  const Eigen::Vector2d dt(xj.x - xi.x, xj.y - xi.y);
  const Eigen::Vector2d et = rot(z.theta).transpose() *
                             (rot(xi.theta).transpose() * dt - Eigen::Vector2d(z.x, z.y));
  return Eigen::Vector3d(et.x(), et.y(), wrapAngle(xj.theta - xi.theta - z.theta));
}

double graphCost(const std::vector<Pose2>& poses, const std::vector<Edge2>& edges) {
  double c = 0.0;
  for (const auto& e : edges) {
    const Eigen::Vector3d r = edgeError(poses[e.i], poses[e.j], e.z);
    c += r.dot(e.info * r);
  }
  return c;
}

int optimizePoseGraph(std::vector<Pose2>& poses, const std::vector<Edge2>& edges, int max_iters = 20) {
  const int N = static_cast<int>(poses.size());
  if (N < 2 || edges.empty()) return 0;
  const int n = 3 * N;
  int it = 0;
  while (it < max_iters) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
    for (const auto& ed : edges) {
      const Pose2& xi = poses[ed.i];
      const Pose2& xj = poses[ed.j];
      const Eigen::Vector3d e = edgeError(xi, xj, ed.z);
      const Eigen::Matrix2d RzT = rot(ed.z.theta).transpose();
      const Eigen::Matrix2d RiT = rot(xi.theta).transpose();
      const double c = std::cos(xi.theta), s = std::sin(xi.theta);
      Eigen::Matrix2d dRiT;  // d(R_i^T)/d(theta_i)
      dRiT << -s, c, -c, -s;
      const Eigen::Vector2d dt(xj.x - xi.x, xj.y - xi.y);

      Eigen::Matrix3d A = Eigen::Matrix3d::Zero(), B = Eigen::Matrix3d::Zero();
      A.topLeftCorner<2, 2>() = -RzT * RiT;
      A.topRightCorner<2, 1>() = RzT * dRiT * dt;
      A(2, 2) = -1.0;
      B.topLeftCorner<2, 2>() = RzT * RiT;
      B(2, 2) = 1.0;

      const Eigen::Matrix3d& O = ed.info;
      const int I = 3 * ed.i, J = 3 * ed.j;
      H.block<3, 3>(I, I) += A.transpose() * O * A;
      H.block<3, 3>(I, J) += A.transpose() * O * B;
      H.block<3, 3>(J, I) += B.transpose() * O * A;
      H.block<3, 3>(J, J) += B.transpose() * O * B;
      b.segment<3>(I) += A.transpose() * O * e;
      b.segment<3>(J) += B.transpose() * O * e;
    }
    // Gauge: pose 0 is fixed -> drop its rows/columns and solve for the rest.
    const Eigen::VectorXd dx =
        H.bottomRightCorner(n - 3, n - 3).ldlt().solve(-b.tail(n - 3));
    if (!dx.allFinite()) break;
    for (int k = 1; k < N; ++k) {
      poses[k].x += dx[3 * (k - 1)];
      poses[k].y += dx[3 * (k - 1) + 1];
      poses[k].theta = wrapAngle(poses[k].theta + dx[3 * (k - 1) + 2]);
    }
    ++it;
    if (dx.norm() < 1e-9) break;
  }
  poses[0].theta = wrapAngle(poses[0].theta);
  return it;
}
