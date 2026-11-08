#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <set>
#include <tuple>
#include <unordered_set>
#include <vector>

struct Point3 { double x, y, z; };
struct FrameDiff {
  std::vector<int> tracked;
  std::vector<int> lost;
  std::vector<int> appeared;
};

FrameDiff diffFrames(const std::vector<int>& prev, const std::vector<int>& curr) {
  const std::set<int> a(prev.begin(), prev.end());  // sorted + unique, O(n log n)
  const std::set<int> b(curr.begin(), curr.end());
  FrameDiff d;
  // The set algorithms do one linear merge-style pass over two SORTED ranges: O(|a| + |b|).
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(d.tracked));
  std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(d.lost));
  std::set_difference(b.begin(), b.end(), a.begin(), a.end(), std::back_inserter(d.appeared));
  return d;
}

std::size_t countUniqueVoxels(const std::vector<Point3>& cloud, double voxel) {
  // std::tuple has a lexicographic operator<, so it works as a std::set key with no extra code.
  std::set<std::tuple<int, int, int>> occupied;
  for (const auto& p : cloud)
    occupied.emplace(static_cast<int>(std::floor(p.x / voxel)), static_cast<int>(std::floor(p.y / voxel)),
                     static_cast<int>(std::floor(p.z / voxel)));
  return occupied.size();
}

bool hasDuplicateIds(const std::vector<int>& ids) {
  std::unordered_set<int> seen;
  seen.reserve(ids.size());
  for (int id : ids)
    if (!seen.insert(id).second) return true;  // .second == false: already present
  return false;
}
