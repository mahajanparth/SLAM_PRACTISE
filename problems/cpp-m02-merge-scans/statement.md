# Scan Buffer: append & merge

A deskewing front-end collects points from several lidar heads, each a time-sorted stream, and hands one merged, time-sorted buffer to the odometry thread. Allocation happens on the hot path, so the buffer code has to be careful with `reserve`, `capacity` and aliasing.

```cpp
struct Point3 { double x, y, z; };
struct StampedPoint { double t; Point3 p; };
```

| Function | Behaviour |
|---|---|
| `void appendScan(std::vector<Point3>& dst, const std::vector<Point3>& src)` | Append all of `src` to the end of `dst` with **at most one reallocation**. Must work when `src` and `dst` are the **same vector** (the cloud doubles itself) |
| `std::vector<StampedPoint> mergeSorted(const std::vector<std::vector<StampedPoint>>& scans)` | Each `scans[k]` is sorted by `t` (non-decreasing). Return all points sorted by `t`. **Ties** go first by lower scan index `k`, then by original position. The result must be allocated **exactly once**: `result.capacity() == result.size()` |

### Constraints
- Total points N ≤ 2·10⁵, number of scans k ≤ 64. Target O(N log k) for the merge (O(N log N) is accepted).
- Empty scans and an empty list of scans are valid inputs.
- Runs under ASan/UBSan.

### Think about
- `push_back` growth is geometric (×2 in libstdc++, ×1.5 in MSVC). Why does that give amortised O(1), and why not grow by +1000?
- Why is `for (const auto& p : src) dst.push_back(p);` broken when `&src == &dst`? Name both bugs.
- `reserve(n)` vs `resize(n)`: what does each do to `size()`, and which constructs elements?
- What invalidates iterators on `std::vector`? Which operations leave them valid if there's no reallocation?
