# k-Nearest Points & k-Way Sensor Merge

Two everyday heap problems in a perception stack.

```cpp
struct Point3 { double x, y, z; };
struct Stamped { double t; int sensor; };
```

### 1. `std::vector<int> kNearest(const std::vector<Point3>& cloud, const Point3& q, int k)`
Indices of the `k` points closest to `q` (Euclidean), ordered by **ascending distance**. Equal distances are
ordered by **smaller index first**. If `k >= cloud.size()`, return all indices (sorted the same way). If `k <= 0`, return `{}`.
Target: **O(n log k)** time, O(k) extra memory. Don't sort the whole cloud.

### 2. `std::vector<Stamped> mergeStreams(const std::vector<std::vector<Stamped>>& streams)`
Each inner vector is one sensor's messages, already sorted by `t` (non-decreasing). Merge them into one sequence sorted
by `t`. On ties, the message from the **lower stream index** comes first, and messages from the same stream keep their order.
Streams may be empty. Target: **O(N log S)** for N messages in S streams.

### Think about
- Max-heap or min-heap for k-nearest, and why?
- When would you use `std::nth_element` instead? What about a KD-tree?
- Why compare squared distances?
- How does `mergeStreams` change if the streams are unbounded live feeds (when is it safe to emit)?
