#include <cmath>

struct Vec2 { double x, y; };

class Rotation2D {
 public:
  Rotation2D() {
    // TODO
  }
  explicit Rotation2D(double theta) {
    // TODO
  }
  static Rotation2D fromCosSin(double c, double s) {
    // TODO
    return Rotation2D();
  }
  static Rotation2D fromVectors(Vec2 from, Vec2 to) {
    // TODO
    return Rotation2D();
  }

  double angle() const { return 0.0; /* TODO */ }
  double cos() const { return c_; }
  double sin() const { return s_; }

  Rotation2D inverse() const {
    // TODO
    return Rotation2D();
  }
  Rotation2D operator*(const Rotation2D& o) const {
    // TODO
    return Rotation2D();
  }
  Vec2 operator*(const Vec2& v) const {
    // TODO
    return {0.0, 0.0};
  }

 private:
  double c_ = 0.0, s_ = 0.0;  // TODO: invariant c_^2 + s_^2 == 1
};
