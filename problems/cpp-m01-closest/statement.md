# Point Utilities

You're given a lidar cloud as `std::vector<Point3>`, where `struct Point3 { double x, y, z; };` is already defined. Implement:

| Function | Behaviour |
|---|---|
| `double range(const Point3& p)` | Euclidean distance from the sensor origin |
| `void scaleInPlace(std::vector<Point3>& cloud, double s)` | Multiply every coordinate by `s`, **without copying** points |
| `const Point3* findClosest(const std::vector<Point3>& cloud, const Point3& q)` | Pointer **into** `cloud` to the point closest to `q`, or `nullptr` if the cloud is empty |
| `double pathLength(const std::vector<Point3>& cloud)` | Length of the polyline `cloud[0] → cloud[1] → … → cloud[n-1]`; returns 0 for fewer than 2 points |

### Constraints
- Up to 10⁶ points; each function must be O(n).
- No undefined behaviour. The tests run under AddressSanitizer and UBSan.

### Think about
- Why return `const Point3*` and not a reference or `std::optional<Point3>`?
- Can `findClosest` avoid computing a square root?
