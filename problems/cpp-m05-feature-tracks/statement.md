# Feature Track Bookkeeping

A visual odometry front-end tracks ORB features between consecutive frames. Each frame reports the ids of the features it currently sees. The list is **unsorted** and may contain **duplicates** because of re-detections. The tracker needs to know which tracks continued, which were lost, and which are new.

```cpp
struct Point3 { double x, y, z; };
struct FrameDiff {
  std::vector<int> tracked;   // in both prev and curr
  std::vector<int> lost;      // in prev, not in curr
  std::vector<int> appeared;  // in curr, not in prev
};
```

| Function | Behaviour |
|---|---|
| `FrameDiff diffFrames(const std::vector<int>& prev, const std::vector<int>& curr)` | All three lists **ascending and duplicate-free** |
| `std::size_t countUniqueVoxels(const std::vector<Point3>& cloud, double voxel)` | Number of distinct occupied voxels $(\lfloor x/v\rfloor, \lfloor y/v\rfloor, \lfloor z/v\rfloor)$ |
| `bool hasDuplicateIds(const std::vector<int>& ids)` | `true` if any id appears twice. Stop at the first duplicate |

### Constraints
- Up to 2·10⁵ ids/points. O(n log n) for `diffFrames`/`countUniqueVoxels`, expected O(n) for `hasDuplicateIds`.

### Think about
- `std::set` vs `std::unordered_set` vs sort + `unique` on a vector: which is fastest here, and why?
- Why do `std::set_intersection` and friends require sorted input, and what do they cost?
- What's the per-element memory overhead of a `std::set<int>`?
- How would you key an `unordered_set` by a voxel `tuple`? (There's no `std::hash<std::tuple>`.)
