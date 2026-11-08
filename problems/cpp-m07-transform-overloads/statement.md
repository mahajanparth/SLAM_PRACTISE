# Transform Utility: overloads & rvalue sinks

Your odometry node moves lidar points from the sensor frame into the odom frame. The pose is planar plus height: a rotation `yaw` about $z$, then a translation $(t_x, t_y, t_z)$:

$$p' = R_z(\text{yaw})\,p + t$$

```cpp
struct Point3 { double x, y, z; };
struct Pose   { double yaw, tx, ty, tz; };
```

Implement this overload set:

| Function | Behaviour |
|---|---|
| `Pose makePose(double yaw, double tx = 0, double ty = 0, double tz = 0)` | Builder with **default arguments** |
| `Point3 apply(const Pose& T, const Point3& p)` | Transform a single point |
| `void applyInPlace(const Pose& T, std::vector<Point3>& cloud)` | Transform every point **in place** |
| `std::vector<Point3> apply(const Pose& T, const std::vector<Point3>& cloud)` | Return a transformed **copy**; `cloud` is untouched |
| `std::vector<Point3> apply(const Pose& T, std::vector<Point3>&& cloud)` | Caller gave up the cloud: transform it in place and return **the same buffer** (no new allocation) |

### Requirements
- The `&&` overload must not allocate. The tests check that `result.data()` equals the original buffer's address.
- The `const&` overload performs exactly one allocation (reserve the size up front).
- Empty clouds work with every overload.

### Think about
- With both `const vector&` and `vector&&` overloads, which one does `apply(T, makeCloud())` pick? What about `apply(T, cloud)`?
- Why is `return cloud;` a copy inside the `&&` overload, while `return local;` would be a move or elided?
- Would a single `apply(const Pose&, std::vector<Point3> cloud)` (by value) replace both overloads? What does that cost?
- Why does `Pose` go in by `const&` while `double yaw` goes in by value?
