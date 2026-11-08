# In-place Cloud Filters

A preprocessing node cleans each lidar scan before registration. Scans are big and arrive at 10–20 Hz, so every filter must work **in place** in **O(n)** time and keep the **original order** of the surviving points.

```cpp
struct Point3 { double x, y, z; };
```

| Function | Behaviour |
|---|---|
| `void removeOutOfRange(std::vector<Point3>& cloud, double min_r, double max_r)` | Keep only points whose range $\sqrt{x^2+y^2+z^2}$ lies in `[min_r, max_r]` (inclusive) |
| `void removeEveryNth(std::vector<Point3>& cloud, std::size_t n)` | Remove the n-th, 2n-th, 3n-th, … elements (1-based: indices n-1, 2n-1, …). `n == 0` means do nothing; `n == 1` empties the cloud |
| `std::size_t dedupConsecutive(std::vector<Point3>& cloud, double eps)` | Collapse runs of consecutive points that lie within `eps` (Euclidean) of the **last kept point**, keeping the first of each run. Return how many points were removed |

### Constraints
- Up to 2·10⁵ points. Each function is O(n), so no `erase` inside a loop.
- Every function keeps the relative order of the survivors.
- Runs under ASan/UBSan, so no invalidated-iterator use.

### Think about
- What does `std::remove_if` actually do to the vector's `size()`? What's in the tail afterwards?
- Why is `for (auto it = v.begin(); it != v.end(); ++it) if (bad(*it)) v.erase(it);` wrong in **two** ways?
- After filtering, `capacity()` hasn't shrunk. When would you call `shrink_to_fit()`, and is it binding?
- If order did **not** matter, how could you remove an element in O(1)?
