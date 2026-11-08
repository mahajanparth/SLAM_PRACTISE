# Voxel Downsampling with a Custom Hash

Before ICP, a 100k-point scan is reduced to one point per $v \times v \times v$ voxel (the centroid of the points inside it). The voxel grid is sparse and unbounded, so you store only the occupied cells in an `unordered_map` keyed by integer voxel coordinates. That means writing the hash yourself.

```cpp
struct Point3 { double x, y, z; };
struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& o) const;
};
struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const;
};
```

| Function | Behaviour |
|---|---|
| `VoxelKey::operator==` | Component-wise equality |
| `VoxelKeyHash::operator()` | A hash that **mixes** all three coordinates. Tested for collisions: over a 40×40×40 block of keys centred on the origin, at least half of the hash values must be distinct |
| `VoxelKey toKey(const Point3& p, double voxel)` | $\big(\lfloor x/v \rfloor, \lfloor y/v \rfloor, \lfloor z/v \rfloor\big)$. Correct for **negative** coordinates |
| `std::vector<Point3> voxelDownsample(const std::vector<Point3>& cloud, double voxel)` | One centroid per occupied voxel, ordered by the voxel's **first appearance** in `cloud` |

Assume `voxel > 0` and coordinates with $|x/v| < 2^{30}$.

### Constraints
- n ≤ 2·10⁵, expected O(n). A degenerate hash makes the map O(n²) and times out.

### Think about
- What's the contract between `operator==` and the hash? What happens if you break it?
- Why does `x ^ y ^ z` collide so badly on a grid? Why is the classic `x*73856093 ^ y*19349663 ^ z*83492791` written with *unsigned* arithmetic?
- Load factor, `reserve`, rehash cost, and pointer/reference stability of `unordered_map` elements across a rehash.
- The alternative without a hash: compute a 64-bit linear index, then `sort` + group. When does that win?
