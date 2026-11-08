# Rotation2D: a Class That Keeps Its Invariant

Planar robots (diff-drive, AGVs) rotate headings all the time, and storing a raw angle leads to wrap-around bugs. Store the rotation as a **unit complex number** $(c, s) = (\cos\theta, \sin\theta)$ and **guarantee** $c^2 + s^2 = 1$ for every object. The given types are `struct Vec2 { double x, y; };` and a class with private `double c_, s_;`.

| Member | Behaviour |
|---|---|
| `Rotation2D()` | Identity. |
| `explicit Rotation2D(double theta)` | Rotation by `theta` radians (any real value). |
| `static Rotation2D fromCosSin(double c, double s)` | Normalises `(c, s)`. If its norm is `< 1e-12`, returns the identity. |
| `static Rotation2D fromVectors(Vec2 from, Vec2 to)` | The rotation that turns the direction of `from` into the direction of `to`. Identity if either vector has zero length (`< 1e-12`). |
| `double angle() const` | The angle in **(−π, π]**. |
| `double cos() const`, `double sin() const` | The stored components. |
| `Rotation2D inverse() const` | The inverse rotation. |
| `Rotation2D operator*(const Rotation2D& o) const` | Composition: `(a * b)` applies `b` first, then `a`. |
| `Vec2 operator*(const Vec2& v) const` | Rotates a vector. |

All query methods must be callable on a `const Rotation2D`. The tests also check that `double` does **not** implicitly convert to `Rotation2D`.

### Constraints
- Every operation is O(1). `std::cmath` only.
- After **any** sequence of operations, $|c^2 + s^2 - 1| < 10^{-9}$.

### Think about
- Why make `Rotation2D(double)` `explicit`? What bug does it prevent in `rotate(v, 90)`?
- Composition of two exact unit numbers drifts in floating point. When would you renormalise?
- Why a **static factory** (`fromCosSin`) instead of a public `Rotation2D(double c, double s)` constructor?
- How does this generalise to unit quaternions for SO(3)?
