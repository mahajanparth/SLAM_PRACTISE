#include <cmath>
#include <vector>

struct Pose2 { double x = 0, y = 0, theta = 0; };
struct OdomSample { double v, omega, dt; };

double wrapAngle(double a) {
  // TODO
  return a;
}

Pose2 compose(const Pose2& a, const Pose2& b) {
  // TODO
  return a;
}

Pose2 integrateOdometry(const Pose2& start, const std::vector<OdomSample>& samples) {
  // TODO
  return start;
}
