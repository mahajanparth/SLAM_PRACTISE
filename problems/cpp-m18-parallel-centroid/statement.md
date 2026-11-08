# Parallel Cloud Reductions

A 128-beam lidar delivers about 2M points per second. Some per-scan statistics are cheap enough to split across a few cores.
Implement three small building blocks with `std::thread`, `std::atomic`, and `std::mutex`:

```cpp
struct Point3 { double x, y, z; };
```

| Function / class | Behaviour |
|---|---|
| `Point3 parallelCentroid(const std::vector<Point3>& cloud, int num_threads)` | Mean of the points, computed by `T` threads each summing one contiguous chunk. `{0,0,0}` if empty |
| `std::size_t countInRangeParallel(const std::vector<Point3>& cloud, double max_range, int num_threads)` | Number of points with Euclidean norm `<= max_range`, counted by `T` threads into a `std::atomic<std::size_t>` |
| `class HitCounter` | Thread-safe map *cell id → hits*: `void add(int cell, int n = 1)`, `int get(int cell) const` (0 if never hit), `std::size_t distinct() const` |

Use `T = clamp(num_threads, 1, cloud.size())`: never more threads than points, and at least 1. The tests call
`HitCounter::add` from several threads at once.

### Rules
- Every thread you start must be joined before the function returns.
- No data races. (The tests run under ASan; the reference compiles cleanly under TSan too.)
- Results must not depend on scheduling: the sum may differ from a serial sum only by floating-point rounding.

### Think about
- Why is a per-thread partial sum better than one shared `std::atomic<double>` or a mutex-protected sum?
- Where can **false sharing** appear in `partial[t]`, and how would you avoid it?
- How many threads should you use? What about thread creation cost vs. a 1000-point cloud?
- How would `std::async` or C++17 `std::reduce(std::execution::par, ...)` change the code?
