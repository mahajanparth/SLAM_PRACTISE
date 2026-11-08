# Cloud Queries with `<algorithm>`

Interviewers like asking for these "without writing a raw loop". Each one maps onto a standard algorithm, and knowing which algorithm, and its complexity, is the point of the exercise.

```cpp
struct Point3 { double x, y, z; };
```

| Function | Behaviour |
|---|---|
| `Point3 centroid(const std::vector<Point3>& c)` | Mean point. Returns `{0,0,0}` for an empty cloud |
| `std::size_t countWithin(const std::vector<Point3>& c, const Point3& q, double r)` | Number of points with $\lVert p - q\rVert \le r$ |
| `void sortByRange(std::vector<Point3>& c)` | Sort ascending by range from the origin. Points with **equal range keep their original relative order** |
| `std::vector<std::size_t> kClosest(const std::vector<Point3>& c, const Point3& q, std::size_t k)` | Indices of the `k` points nearest to `q`, ordered by increasing distance. Ties go to the **smaller index**. If `k > c.size()`, return all of them |

### Constraints
- n ≤ 2·10⁵. `kClosest` should be O(n + k log k) (`nth_element`), not O(n log n).

### Think about
- `sort` vs `stable_sort` vs `partial_sort` vs `nth_element`: guarantees and complexity of each.
- Why must a comparator be a strict weak ordering? What can `std::sort` do with `<=`?
- `std::accumulate` vs `std::reduce`: what changes for floating point?
- For repeated kNN queries on the same cloud, what would you build instead?
