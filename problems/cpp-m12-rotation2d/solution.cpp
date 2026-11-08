#include <cmath>

struct Vec2 { double x, y; };

class Rotation2D {
 public:
  Rotation2D() = default;  // identity via default member initialisers
  explicit Rotation2D(double theta) : c_(std::cos(theta)), s_(std::sin(theta)) {}

  // Static factory: the name documents the meaning of the arguments, and it can
  // normalise / handle degenerate input before the object exists.
  static Rotation2D fromCosSin(double c, double s) {
    const double n = std::hypot(c, s);
    if (n < 1e-12) return Rotation2D();
    return Rotation2D(c / n, s / n, Unchecked{});
  }

  static Rotation2D fromVectors(Vec2 from, Vec2 to) {
    const double nf = std::hypot(from.x, from.y), nt = std::hypot(to.x, to.y);
    if (nf < 1e-12 || nt < 1e-12) return Rotation2D();
    // cos = from·to, sin = from×to (z-component); fromCosSin normalises by |from||to|.
    return fromCosSin(from.x * to.x + from.y * to.y, from.x * to.y - from.y * to.x);
  }

  double angle() const {
    const double a = std::atan2(s_, c_);       // [-pi, pi]
    return a <= -M_PI ? a + 2.0 * M_PI : a;    // (-pi, pi]
  }
  double cos() const { return c_; }
  double sin() const { return s_; }

  Rotation2D inverse() const { return Rotation2D(c_, -s_, Unchecked{}); }

  Rotation2D operator*(const Rotation2D& o) const {
    // Complex multiplication; renormalise so the invariant survives long chains.
    return fromCosSin(c_ * o.c_ - s_ * o.s_, s_ * o.c_ + c_ * o.s_);
  }

  Vec2 operator*(const Vec2& v) const { return {c_ * v.x - s_ * v.y, s_ * v.x + c_ * v.y}; }

 private:
  struct Unchecked {};  // tag: caller guarantees (c, s) is already unit length
  Rotation2D(double c, double s, Unchecked) : c_(c), s_(s) {}

  double c_ = 1.0, s_ = 0.0;  // invariant: c_^2 + s_^2 == 1
};
