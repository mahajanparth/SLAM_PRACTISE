#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

struct Point3 { double x, y, z; };

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

// Equal keys MUST produce equal hashes. Distinct keys should rarely collide.
// hash_combine (boost-style): the golden-ratio constant and shifts spread bits, so (1,2,3) != (3,2,1).
struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const {
    std::size_t h = 0;
    for (int v : {k.x, k.y, k.z})
      h ^= std::hash<int>{}(v) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
    // Alternative (Teschner et al.), done in unsigned to avoid signed-overflow UB:
    // return (uint64_t(uint32_t(k.x)) * 73856093u) ^ (uint64_t(uint32_t(k.y)) * 19349663u) ^
    //        (uint64_t(uint32_t(k.z)) * 83492791u);
  }
};

VoxelKey toKey(const Point3& p, double voxel) {
  // floor, NOT truncation: -0.3/0.5 = -0.6 must map to -1, not 0.
  return VoxelKey{static_cast<int>(std::floor(p.x / voxel)), static_cast<int>(std::floor(p.y / voxel)),
                  static_cast<int>(std::floor(p.z / voxel))};
}

std::vector<Point3> voxelDownsample(const std::vector<Point3>& cloud, double voxel) {
  struct Acc { double sx = 0, sy = 0, sz = 0; std::size_t n = 0; };
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> slot;  // key -> index into acc
  slot.reserve(cloud.size());                                     // avoid rehashes while inserting
  std::vector<Acc> acc;

  for (const auto& p : cloud) {
    // try_emplace inserts only if absent and returns {iterator, inserted}: one hash lookup.
    auto [it, inserted] = slot.try_emplace(toKey(p, voxel), acc.size());
    if (inserted) acc.emplace_back();
    Acc& a = acc[it->second];
    a.sx += p.x; a.sy += p.y; a.sz += p.z; ++a.n;
  }

  std::vector<Point3> out;
  out.reserve(acc.size());
  for (const auto& a : acc) out.push_back({a.sx / a.n, a.sy / a.n, a.sz / a.n});
  return out;  // order = first appearance (slots were appended in input order)
}
