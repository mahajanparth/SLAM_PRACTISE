# LidarScan Builder

A driver assembles `LidarScan` messages at 20 Hz with thousands of ranges each. Copying a scan by accident on the hot path shows up straight away in the profile. Build an API that **never copies when the caller doesn't need it to**.

```cpp
struct LidarScan {
  double stamp = 0.0;
  std::string frame_id;
  std::vector<double> ranges;
};

class ScanBuilder {
 public:
  ScanBuilder& stamp(double t);
  ScanBuilder& frame(std::string id);                    // sink parameter
  ScanBuilder& addRange(double r);
  ScanBuilder& addRanges(const std::vector<double>& rs); // append, keep order
  const std::vector<double>& ranges() const;             // read-only view
  LidarScan build() const&;   // builder still usable afterwards: copies
  LidarScan build() &&;       // builder is expiring: moves its buffers out
 private:
  double stamp_ = 0.0;
  std::string frame_;
  std::vector<double> ranges_;
};

std::vector<double> clampRanges(std::vector<double> r, double min_r = 0.1, double max_r = 100.0);
```

### Semantics
- Setters return `*this`, so calls chain: `ScanBuilder().stamp(1.0).frame("lidar").addRange(3.0)`.
- `build() const&` (called on an lvalue builder) leaves the builder unchanged, so you can call it again.
- `build() &&` (called on a temporary or after `std::move(b)`) must hand over the builder's range buffer. The tests check that `scan.ranges.data()` equals the address `b.ranges().data()` had before the build.
- `clampRanges` clamps every value into `[min_r, max_r]` (`std::clamp`). When the caller passes `std::move(v)`, the result must reuse `v`'s buffer.

### Think about
- When does `ScanBuilder().stamp(1).build()` pick the `&&` overload? Does the chain keep it an rvalue? (Hint: `stamp` returns `ScanBuilder&`.)
- Sink by value vs a `const std::string&` + `std::string&&` overload pair: how many copies/moves does each cost for an lvalue and for an rvalue?
- When is RVO *guaranteed* in C++17 (prvalue), and when is it only *permitted* (NRVO)?
- Why is `return std::move(local);` a pessimization?
