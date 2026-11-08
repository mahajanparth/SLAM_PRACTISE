// The candidate's code is injected before this file (g++ -include).
#include <random>
#include "common/check.h"

static Pose2 compose(const Pose2& a, const Pose2& b) {
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.theta + b.theta};  // theta NOT wrapped
}
static Pose2 between(const Pose2& a, const Pose2& b) {
  const double c = std::cos(a.theta), s = std::sin(a.theta), dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy, -s * dx + c * dy, std::atan2(std::sin(b.theta - a.theta), std::cos(b.theta - a.theta))};
}
static double angDiff(double a, double b) { return std::abs(std::atan2(std::sin(a - b), std::cos(a - b))); }

struct Graph { std::vector<Pose2> gt, init; std::vector<Edge2> edges; };

// Square loop: `side` unit steps per side, 4 sides, last pose closes back to pose 0.
static Graph squareLoop(int side, double sig_t, double sig_th, double th_bias, unsigned seed, bool loop = true) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> g(0.0, 1.0);
  Graph G;
  const int N = 4 * side;
  G.gt.push_back({0, 0, 0});
  G.init.push_back({0, 0, 0});
  Eigen::Matrix3d odo_info = Eigen::Vector3d(1 / (sig_t * sig_t + 1e-6), 1 / (sig_t * sig_t + 1e-6),
                                             1 / (sig_th * sig_th + 1e-6)).asDiagonal();
  for (int k = 0; k + 1 < N; ++k) {
    const Pose2 u{1.0, 0.0, (k % side == side - 1) ? M_PI / 2 : 0.0};
    G.gt.push_back(compose(G.gt.back(), u));
    const Pose2 zn{u.x + sig_t * g(rng), u.y + sig_t * g(rng), u.theta + th_bias + sig_th * g(rng)};
    G.init.push_back(compose(G.init.back(), zn));  // dead reckoning, unwrapped headings
    G.edges.push_back({k, k + 1, zn, odo_info});
  }
  if (loop) {
    Edge2 lc{N - 1, 0, between(G.gt[N - 1], G.gt[0]), Eigen::Matrix3d::Identity()};
    lc.info = Eigen::Vector3d(1e3, 1e3, 1e4).asDiagonal();
    G.edges.push_back(lc);
  }
  return G;
}

static double maxPosErr(const std::vector<Pose2>& a, const std::vector<Pose2>& b) {
  double m = 0;
  for (std::size_t k = 0; k < a.size(); ++k) m = std::max(m, std::hypot(a[k].x - b[k].x, a[k].y - b[k].y));
  return m;
}

int main() {
  // edgeError on hand-computed values.
  const Eigen::Vector3d e = edgeError({1, 2, M_PI / 2}, {1, 4, M_PI / 2}, {1.5, 0.5, 0.1});
  CHECK_MAT_NEAR(e, Eigen::Vector3d(std::cos(0.1) * 0.5 - std::sin(0.1) * 0.5,
                                    -std::sin(0.1) * 0.5 - std::cos(0.1) * 0.5, -0.1), 1e-12);

  Graph G = squareLoop(4, 0.03, 0.01, 0.02, 11);
  std::vector<Pose2> poses = G.init;
  const Edge2& lc = G.edges.back();
  const double lc_before = edgeError(poses[lc.i], poses[lc.j], lc.z).norm();
  const double cost_before = graphCost(poses, G.edges);
  CHECK(cost_before > 100.0);
  const int iters = optimizePoseGraph(poses, G.edges);
  CHECK(iters >= 1 && iters <= 20);
  const double lc_after = edgeError(poses[lc.i], poses[lc.j], lc.z).norm();
  CHECK(lc_after < 0.1 * lc_before);
  CHECK(graphCost(poses, G.edges) < 0.05 * cost_before);
  CHECK(maxPosErr(poses, G.gt) < 0.15);
  CHECK(maxPosErr(G.init, G.gt) > 0.5);  // (sanity: the initial guess really drifted)
  CHECK(std::abs(poses[0].x) < 1e-6 && std::abs(poses[0].y) < 1e-6 && std::abs(poses[0].theta) < 1e-6);

#ifdef HIDDEN
  // Angle wrap inside the residual.
  CHECK_NEAR(edgeError({0, 0, 3.0}, {0, 0, -3.0}, {0, 0, 0})[2], 2 * M_PI - 6.0, 1e-12);
  CHECK_NEAR(edgeError({0, 0, 0}, {0, 0, 0}, {0, 0, 2 * M_PI})[2], 0.0, 1e-12);

  // Headings returned wrapped & close to ground truth.
  bool wrapped = true, head_ok = true;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    wrapped = wrapped && std::abs(poses[k].theta) <= M_PI + 1e-9;
    head_ok = head_ok && angDiff(poses[k].theta, G.gt[k].theta) < 0.05;
  }
  CHECK(wrapped);
  CHECK(head_ok);

  // Perfect initial guess + exact measurements: nothing moves, cost 0, converges fast.
  Graph P = squareLoop(3, 0.0, 0.0, 0.0, 1);
  std::vector<Pose2> pp = P.gt;
  const int it2 = optimizePoseGraph(pp, P.edges);
  CHECK(it2 <= 2);
  CHECK(maxPosErr(pp, P.gt) < 1e-9);
  CHECK_NEAR(graphCost(pp, P.edges), 0.0, 1e-12);

  // Odometry only (no loop closure): optimum = dead reckoning, cost 0.
  Graph O = squareLoop(5, 0.05, 0.02, 0.03, 4, /*loop=*/false);
  std::vector<Pose2> po(O.init.size(), Pose2{});  // start everything at the origin
  optimizePoseGraph(po, O.edges);
  CHECK(maxPosErr(po, O.init) < 1e-6);
  CHECK_NEAR(graphCost(po, O.edges), 0.0, 1e-9);

  // Bigger, noisier loop with strong heading drift (unwrapped init headings well beyond 2*pi).
  Graph Bg = squareLoop(12, 0.02, 0.01, 0.04, 23);
  std::vector<Pose2> pb = Bg.init;
  CHECK(pb.back().theta > 2 * M_PI && maxPosErr(pb, Bg.gt) > 3.0);
  optimizePoseGraph(pb, Bg.edges, 30);
  // A global optimum can be no worse than the ground truth configuration.
  CHECK(graphCost(pb, Bg.edges) <= graphCost(Bg.gt, Bg.edges) + 1e-6);
  CHECK(maxPosErr(pb, Bg.gt) < 1.0);
  CHECK(std::abs(pb[0].x) < 1e-6 && std::abs(pb[0].theta) < 1e-6);

  // Degenerate: no edges / single pose -> untouched, 0 iterations.
  std::vector<Pose2> one = {{1, 2, 3}};
  CHECK_EQ(optimizePoseGraph(one, {}), 0);
  CHECK_NEAR(one[0].theta, 3.0, 0.0);
  std::vector<Pose2> two = {{0, 0, 0}, {5, 5, 1}};
  CHECK_EQ(optimizePoseGraph(two, {}), 0);
  CHECK_NEAR(two[1].x, 5.0, 0.0);
#endif
  return TEST_SUMMARY();
}
