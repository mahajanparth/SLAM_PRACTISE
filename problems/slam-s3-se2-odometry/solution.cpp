#include <cmath>
#include <vector>

struct Pose2 { double x = 0, y = 0, theta = 0; };
struct OdomSample { double v, omega, dt; };

double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Pose2 compose(const Pose2& a, const Pose2& b) {
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrapAngle(a.theta + b.theta)};
}

Pose2 integrateOdometry(const Pose2& start, const std::vector<OdomSample>& samples) {
  Pose2 p{start.x, start.y, wrapAngle(start.theta)};
  for (const auto& s : samples) {
    const double dth = s.omega * s.dt;
    if (std::abs(s.omega) < 1e-9) {
      // Straight-line limit (v/omega would blow up). Using the midpoint heading keeps O(omega^2) accuracy.
      const double th = p.theta + 0.5 * dth;
      p.x += s.v * s.dt * std::cos(th);
      p.y += s.v * s.dt * std::sin(th);
    } else {
      // Exact arc of radius v/omega.
      const double r = s.v / s.omega;
      p.x += r * (std::sin(p.theta + dth) - std::sin(p.theta));
      p.y -= r * (std::cos(p.theta + dth) - std::cos(p.theta));
    }
    p.theta = wrapAngle(p.theta + dth);
  }
  return p;
}
